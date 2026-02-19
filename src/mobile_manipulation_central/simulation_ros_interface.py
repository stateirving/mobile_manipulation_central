from rclpy.node import Node
from builtin_interfaces.msg import Time
import numpy as np

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, TransformStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

from mobile_manipulation_central.ros_utils import UR10_JOINT_NAMES


class SimulatedViconObjectInterface:
    """Simulation of the Vicon ROS2 end point for a detected object.

    This is intended to be instantiated from a simulation environment to
    publish data in the same manner as Vicon would do in the real world.
    """

    def __init__(self, node: Node, name):
        topic = "/vicon/" + name + "/" + name
        self.pub = node.create_publisher(TransformStamped, topic, 1)
        self.ground_truth_pub = node.create_publisher(
            JointState, "/projectile/true_joint_states", 1
        )
        self.node = node

    def publish_pose(self, t, r, Q):
        """Publish the object's pose at time t, consisting of position r and
        orientation (represented as a quaternion) Q.

        Note that the order of Q is [x, y, z, w]."""
        msg = TransformStamped()
        # Convert simulation time (float seconds) to ROS2 Time message
        seconds = int(t)
        nanoseconds = int((t - seconds) * 1e9)
        msg.header.stamp = Time(sec=seconds, nanosec=nanoseconds)

        msg.transform.translation.x = r[0]
        msg.transform.translation.y = r[1]
        msg.transform.translation.z = r[2]

        msg.transform.rotation.x = Q[0]
        msg.transform.rotation.y = Q[1]
        msg.transform.rotation.z = Q[2]
        msg.transform.rotation.w = Q[3]

        self.pub.publish(msg)

    def publish_ground_truth(self, t, r, v):
        """Publish ground-truth pose and twist.

        This is useful for debugging purposes: compare estimated state to this
        ground truth."""
        msg = JointState()
        # Convert simulation time (float seconds) to ROS2 Time message
        seconds = int(t)
        nanoseconds = int((t - seconds) * 1e9)
        msg.header.stamp = Time(sec=seconds, nanosec=nanoseconds)
        msg.name = ["x", "y", "z"]
        msg.position = list(r)
        msg.velocity = list(v)
        self.ground_truth_pub.publish(msg)


class SimulatedRobotROSInterface:
    """Interface between the MPC node and a simulated robot.

    This can be used as a generic ROS2 end point to simulate a robot. The idea
    is that a simulator should instantiate this class and update it at the
    desired frequency as the simulation runs.
    """

    def __init__(self, node: Node, nq, nv, robot_name, joint_names):
        self.cmd_vel = None
        self.nq = nq
        self.nv = nv
        self.joint_names = joint_names
        self.node = node

        self.clock_pub = node.create_publisher(Clock, "/clock", 1)
        self.feedback_pub = node.create_publisher(
            JointState, robot_name + "/joint_states", 1
        )

    def ready(self):
        return self.cmd_vel is not None

    def publish_feedback(self, t, q, v):
        assert q.shape == (self.nq,)
        assert v.shape == (self.nv,)

        msg = JointState()
        # Convert simulation time (float seconds) to ROS2 Time message
        seconds = int(t)
        nanoseconds = int((t - seconds) * 1e9)
        msg.header.stamp = Time(sec=seconds, nanosec=nanoseconds)
        msg.name = self.joint_names
        msg.position = list(q)
        msg.velocity = list(v)
        self.feedback_pub.publish(msg)

    def publish_time(self, t):
        """Publish (simulation) time."""
        msg = Clock()
        # Convert simulation time (float seconds) to ROS2 Time message
        seconds = int(t)
        nanoseconds = int((t - seconds) * 1e9)
        msg.clock = Time(sec=seconds, nanosec=nanoseconds)
        self.clock_pub.publish(msg)


class SimulatedRidgebackROSInterface(SimulatedRobotROSInterface):
    """Simulated Ridgeback interface."""

    def __init__(self, node: Node):
        robot_name = "ridgeback"
        super().__init__(
            node=node, nq=3, nv=3, robot_name=robot_name, joint_names=["x", "y", "yaw"]
        )

        self.cmd_sub = node.create_subscription(
            Twist, robot_name + "/cmd_vel", self._cmd_cb, 1
        )

    def _cmd_cb(self, msg):
        self.cmd_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z])


class SimulatedUR10ROSInterface(SimulatedRobotROSInterface):
    """Simulated UR10 interface."""

    def __init__(self, node: Node):
        robot_name = "ur10"
        super().__init__(
            node=node, nq=6, nv=6, robot_name=robot_name, joint_names=UR10_JOINT_NAMES
        )

        self.cmd_sub = node.create_subscription(
            Float64MultiArray, robot_name + "/cmd_vel", self._cmd_cb, 1
        )
        self.node = node

    def _cmd_cb(self, msg):
        self.cmd_vel = np.array(msg.data)
        assert self.cmd_vel.shape == (self.nv,)


class SimulatedMobileManipulatorROSInterface:
    def __init__(self, node: Node):
        self.arm = SimulatedUR10ROSInterface(node=node)
        self.base = SimulatedRidgebackROSInterface(node=node)

        self.nq = self.arm.nq + self.base.nq
        self.nv = self.arm.nv + self.base.nv

    @property
    def cmd_vel(self):
        return np.concatenate((self.base.cmd_vel, self.arm.cmd_vel))

    def ready(self):
        return self.base.ready() and self.arm.ready()

    def publish_feedback(self, t, q, v):
        assert q.shape == (self.nq,)
        assert v.shape == (self.nv,)

        self.base.publish_feedback(t=t, q=q[: self.base.nq], v=v[: self.base.nv])
        self.arm.publish_feedback(t=t, q=q[self.base.nq :], v=v[self.base.nv :])

    def publish_time(self, t):
        """Publish (simulation) time."""
        # arbitrary: we could also use the arm component
        self.base.publish_time(t)
