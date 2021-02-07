"""
ROS controller commands.
"""

# pyright: reportInvalidTypeVarUse=false

from dataclasses import dataclass
from typing import Callable, Dict, Generic, Tuple, Type, TypeVar

import rospy

RosMsg = TypeVar("RosMsg", bound=rospy.Message)

K = TypeVar("K")
V = TypeVar("V")


@dataclass
class Cmd(Generic[RosMsg]):
    """
    A command to specify outbound ROS messages.
    """

    topic_name: str
    message_type: Type[RosMsg]  # no existential types rip
    message_value: RosMsg
    latch_publisher: bool = False


class Publishers:
    """
    A group of ROS publishers to transmit commands.
    """

    pub_dict: Dict[str, rospy.Publisher] = {}

    def publish(self, cmd: Cmd[RosMsg]) -> None:
        """
        Publish the given command and memoize new publishers.
        """
        (pub_dict, pub) = get_or_def(
            kvs=self.pub_dict,
            key=cmd.topic_name,
            mk_def=lambda: mk_ros_pub(cmd),
        )

        self.pub_dict = pub_dict
        pub.publish(cmd.message_value)


def mk_ros_pub(cmd: Cmd[RosMsg]) -> rospy.Publisher:
    """
    Make a ROS publisher to serve the given command.
    """
    return rospy.Publisher(
        name=cmd.topic_name,
        data_class=cmd.message_type,
        latch=cmd.latch_publisher,
        queue_size=10,
    )


def get_or_def(
    kvs: Dict[K, V],
    key: K,
    mk_def: Callable[[], V],
) -> Tuple[Dict[K, V], V]:
    value = kvs.get(key)

    if value is None:
        value_def = mk_def()
        return ({key: value_def, **kvs}, value_def)

    return (kvs, value)
