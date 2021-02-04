"""
ROS controller commands.
"""

# pyright: reportInvalidTypeVarUse=false

from dataclasses import dataclass
from typing import Dict, Generic, Type, TypeVar

import rospy

RosMsg = TypeVar("RosMsg", bound=rospy.Message)


@dataclass
class Cmd(Generic[RosMsg]):
    """
    A command to specify outbound ROS messages.
    """

    topic_name: str
    message_type: Type[RosMsg]  # no existential types rip
    message_value: RosMsg


class Publishers:
    """
    A group of ROS publishers to transmit commands.
    """

    pub_dict: Dict[str, rospy.Publisher] = {}

    def publish(self, cmd: Cmd[RosMsg]) -> None:
        """
        Publish the given command and memoize new publishers.
        """
        publisher = self.pub_dict.setdefault(cmd.topic_name, mk_ros_pub(cmd))
        publisher.publish(cmd.message_value)


def mk_ros_pub(cmd: Cmd[RosMsg]) -> rospy.Publisher:
    """
    Make a ROS publisher to serve the given command.
    """
    return rospy.Publisher(
        name=cmd.topic_name,
        data_class=cmd.message_type,
        queue_size=10,
    )
