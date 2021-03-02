"""
TurtleBot3 pose.
"""

# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass

from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from rospy_util.vector2 import Vector2
import rospy_util.vector2 as v2


@dataclass
class TurtlePose:
    """
    A simplified pose for representing TurtleBot3.
    """

    position: Vector2
    yaw: float


def from_pose(pose: Pose) -> TurtlePose:
    """
    Convert a ROS pose message to a TurtlePose.
    """
    return TurtlePose(
        position=v2.from_point(pose.position),
        yaw=yaw_from_quaternion(pose.orientation),
    )


def to_pose(pose: TurtlePose) -> Pose:
    """
    Convert a TurtlePose to a ROS pose message.
    """
    return Pose(
        position=v2.to_point(pose.position),
        orientation=yaw_to_quaternion(pose.yaw),
    )


def yaw_from_quaternion(q: Quaternion) -> float:
    """
    Extract the yaw, in Euler angles, from a quaternion.
    """

    (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

    return yaw


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """
    Create a quaternion from the given yaw component in Euler angles.
    """
    components = quaternion_from_euler(ai=0.0, aj=0.0, ak=yaw)
    return Quaternion(*components)
