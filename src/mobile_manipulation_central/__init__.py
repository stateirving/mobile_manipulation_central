from .ros_utils import msg_time, parse_time, parse_ur10_joint_state_msgs, trim_msgs
from .ros_interface import (
    ViconObjectInterface,
    RidgebackROSInterface,
    UR10ROSInterface,
    MobileManipulatorROSInterface,
    RobotSignalHandler,
    SimpleSignalHandler,
)
from .simulation_ros_interface import (
    SimulatedRidgebackROSInterface,
    SimulatedUR10ROSInterface,
    SimulatedMobileManipulatorROSInterface,
    SimulatedViconObjectInterface,
)
from .kinematics import MobileManipulatorKinematics
from .trajectory_generation import PointToPointTrajectory, QuinticTimeScaling
from .smoothing import ExponentialSmoother
from .simulation import BulletSimulation, BulletSimulatedRobot
from .ros_logging import BAG_DIR, DataRecorder, ViconRateChecker
from .utils import wrap_to_pi, load_home_position, load_pkg_config

def bound_array(a, lb=None, ub=None):
    """Elementwise bound array above and below."""
    if lb is not None:
        a = np.maximum(a, lb)
    if ub is not None:
        a = np.minimum(a, ub)
    return a