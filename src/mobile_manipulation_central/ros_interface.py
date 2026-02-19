import time
import signal
import threading

import numpy as np
import rclpy
from rclpy.node import Node

from spatialmath.base import rotz
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import TransformStamped

from mobile_manipulation_central import ros_utils


# TODO add protections if time since last message is too large
class JoystickButtonInterface:
    """
        Monitor on joy stick button. Flag event when the button is pressed. Event flag can only be cleared externally.
    """

    def __init__(self, node: Node, button_index):
        self.button_index = button_index
        self.button = 0             # 1 pressed, 0 available
        self.busy = False
        self.button_lock = threading.Lock()
        self.block_out_time = 0.5 # 0.5 second
        self.node = node
        self.last_reset_time = node.get_clock().now().nanoseconds / 1e9

        self.msg_received = False
        self.joy_sub = node.create_subscription(Joy, "/bluetooth_teleop/joy", self._joy_cb, 10)


    def _joy_cb(self, msg):
        if msg.buttons[self.button_index] == 1:
            self._update_button(1)
            print("set button {}".format(self.button))


        self.msg_received = True

    def reset_button(self):
        self._update_button(0, True)
        print("reset button {}".format(self.button))
        self.last_reset_time = self.node.get_clock().now().nanoseconds / 1e9


    def _update_button(self, value, force=False):
        t_now = self.node.get_clock().now().nanoseconds / 1e9
        if t_now - self.last_reset_time > self.block_out_time or force:
            if value != self.button:
                self.button_lock.acquire()
                self.button = value
                print("update button {}".format(self.button))

                self.button_lock.release()

    def ready(self):
        """True if a joy message has been received."""
        return self.msg_received


class ViconObjectInterface:
    """ROS2 interface for receiving Vicon measurements for an object's pose."""

    def __init__(self, node: Node, name):
        topic = "/vicon/" + name + "/" + name
        self.msg_received = False
        self.node = node
        self.sub = node.create_subscription(TransformStamped, topic, self._transform_cb, 1)

    def ready(self):
        """True if a Vicon message has been received."""
        return self.msg_received

    def _transform_cb(self, msg):
        L = msg.transform.translation
        Q = msg.transform.rotation

        self.position = np.array([L.x, L.y, L.z])
        self.orientation = np.array([Q.x, Q.y, Q.z, Q.w])

        self.msg_received = True


# TODO make abstract
class RobotROSInterface:
    """Base class for defining ROS2 interfaces for robots."""

    def __init__(self, node: Node, nq, nv):
        self.node = node
        self.nq = nq
        self.nv = nv

        self.q = np.zeros(self.nq)
        self.v = np.zeros(self.nv)

        self.joint_states_received = False

    def brake(self):
        """Brake (stop) the robot."""
        self.publish_cmd_vel(np.zeros(self.nv))

    def ready(self):
        """True if joint state messages have been received."""
        return self.joint_states_received


class RidgebackROSInterface(RobotROSInterface):
    """ROS2 interface for the Ridgeback mobile base."""

    def __init__(self, node: Node):
        super().__init__(node=node, nq=3, nv=3)

        self.cmd_pub = node.create_publisher(Twist, "/ridgeback/cmd_vel", 1)
        self.joint_state_sub = node.create_subscription(
            JointState, "/ridgeback/joint_states", self._joint_state_cb, 1
        )

    def _joint_state_cb(self, msg):
        """Callback for Ridgeback joint feedback."""
        print("Received Ridgeback joint state message.", flush=True)
        self.q = np.array(msg.position)
        self.v = np.array(msg.velocity)

        self.joint_states_received = True

    def publish_cmd_vel(self, cmd_vel, bodyframe=False):
        """Command the velocity of the robot's joints.

        Setting bodyframe to True indicated that the command is in the base's
        body frame; False indicates it is in the world frame.
        """
        assert cmd_vel.shape == (self.nv,)

        # rotate into body frame from world frame if needed
        if not bodyframe:
            C_bw = rotz(-self.q[2])
            cmd_vel = C_bw @ cmd_vel

        msg = Twist()
        msg.linear.x = cmd_vel[0]
        msg.linear.y = cmd_vel[1]
        msg.angular.z = cmd_vel[2]
        self.cmd_pub.publish(msg)


class UR10ROSInterface(RobotROSInterface):
    """ROS2 interface for the UR10 arm."""

    def __init__(self, node: Node):
        super().__init__(node=node, nq=6, nv=6)

        self.cmd_pub = node.create_publisher(Float64MultiArray, "/ur10/cmd_vel", 1)
        self.joint_state_sub = node.create_subscription(
            JointState, "/ur10/joint_states", self._joint_state_cb, 1
        )

    def _joint_state_cb(self, msg):
        """Callback for arm joint feedback."""
        print("Received UR10 joint state message.", flush=True)
        _, self.q, self.v = ros_utils.parse_ur10_joint_state_msg(msg)
        self.joint_states_received = True

    def publish_cmd_vel(self, cmd_vel, bodyframe=None):
        """Command the velocity of the robot's joints.

        The bodyframe option changes nothing, but is provided for compatibility
        with the Ridgeback interface.
        """
        assert cmd_vel.shape == (self.nv,)

        msg = Float64MultiArray()
        msg.data = list(cmd_vel)
        self.cmd_pub.publish(msg)


class MobileManipulatorROSInterface:
    """ROS2 interface to the real mobile manipulator."""

    def __init__(self, node: Node):
        self.node = node
        self.arm = UR10ROSInterface(node)
        self.base = RidgebackROSInterface(node)

        self.nq = self.arm.nq + self.base.nq
        self.nv = self.arm.nv + self.base.nv

    def brake(self):
        """Brake (stop) the robot."""
        self.base.brake()
        self.arm.brake()

    def ready(self):
        """True if joint state messages have been received for both arm and base."""
        print(f"Base ready: {self.base.ready()}, Arm ready: {self.arm.ready()}", flush=True)
        return self.base.ready() and self.arm.ready()

    def publish_cmd_vel(self, cmd_vel, bodyframe=False):
        """Command the velocity of the robot's joints.

        Setting bodyframe to True indicated that the command is in the base's
        body frame; False indicates it is in the world frame.
        """
        assert cmd_vel.shape == (self.nv,)

        self.base.publish_cmd_vel(cmd_vel[: self.base.nv], bodyframe=bodyframe)
        self.arm.publish_cmd_vel(cmd_vel[self.base.nv :])

    @property
    def q(self):
        """Latest joint configuration measurement."""
        return np.concatenate((self.base.q, self.arm.q))

    @property
    def v(self):
        """Latest joint velocity measurement.

        Note that the base velocity is in the world frame.
        """
        return np.concatenate((self.base.v, self.arm.v))


# TODO: sometimes this hangs at shutdown
# usually this occurs in rospy.impl.registration.RegManager.cleanup, in the
# call to `multi`
class RobotSignalHandler:
    """Custom signal handler to brake the robot before shutting down ROS2."""

    def __init__(self, robot, dry_run=False):
        self.robot = robot
        self.dry_run = dry_run
        signal.signal(signal.SIGINT, self.handler)
        signal.signal(signal.SIGTERM, self.handler)

    def handler(self, signum, frame):
        print("Received SIGINT.")
        if not self.dry_run:
            print("Braking robot.")
            self.robot.brake()
            time.sleep(0.1)  # TODO necessary?
        rclpy.shutdown()


class SimpleSignalHandler:
    """Simple signal handler that just sets a flag when a signal has been caught.

    Parameters
    ----------
    sigint : bool
        Catch SIGINT if ``True`` (the default).
    sigterm : bool
        Catch SIGTERM if ``True`` (default is ``False``).

    Attributes
    ----------
    received : bool
        Initially ``False``; becomes ``True`` once one of the signals has been
        received.
    """

    def __init__(self, sigint=True, sigterm=False, callback=None):
        self.received = False
        self._callback = callback
        if sigint:
            signal.signal(signal.SIGINT, self.handler)
        if sigterm:
            signal.signal(signal.SIGTERM, self.handler)

    def handler(self, signum, frame):
        print(f"Received signal: {signum}")
        self.received = True
        if self._callback is not None:
            self._callback()
