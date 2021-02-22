"""
ROS controller subscriptions.
"""

# pyright: reportInvalidTypeVarUse=false

from dataclasses import dataclass
from typing import Any, Callable, Dict, Generic, List, Type, TypeVar

import rospy

__all__ = (
    "Sub",
    "Subscribers",
    "none",
)

RosMsg = TypeVar("RosMsg", bound=rospy.Message)
Msg = TypeVar("Msg")


@dataclass
class Sub(Generic[RosMsg, Msg]):
    """
    A subscription to specify inbound ROS messages.
    """

    topic_name: str
    message_type: Type[RosMsg]
    to_msg: Callable[[RosMsg], Msg]


class Subscribers(Generic[Msg]):
    """
    A group of ROS subscribers to handle subscriptions.
    """

    sub_dict: Dict[str, rospy.Subscriber] = {}

    def __init__(self, callback: Callable[[Msg], None]) -> None:
        self.callback = callback

    # no existential quantification rip
    def subscribe(self, subs: List[Sub[Any, Msg]]) -> None:
        """
        Register subscribers for new subscriptions, and unregister subscribers
        for subscriptions no longer present.
        """
        topic_names = list(self.sub_dict.keys())

        for sub in subs:
            if sub.topic_name not in topic_names:
                self.sub_dict[sub.topic_name] = mk_ros_sub(sub, self.callback)

        for topic in topic_names:
            if not any(topic == sub.topic_name for sub in subs):
                self.sub_dict.pop(topic).unregister()


none: List[Sub[Any, Any]] = []


def mk_ros_sub(
    sub: Sub[RosMsg, Msg],
    callback: Callable[[Msg], None],
) -> rospy.Subscriber:
    """
    Make a ROS subscriber to run a callback for the given subscription.
    """
    return rospy.Subscriber(
        name=sub.topic_name,
        data_class=sub.message_type,
        callback=lambda t: callback(sub.to_msg(t)),
    )
