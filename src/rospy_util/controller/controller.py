"""
A controller for ROS devices. Inspired by how Elm handles effects, see:
https://guide.elm-lang.org/effects/
"""

from queue import Queue
from threading import Thread
from typing import Any, Callable, Generic, List, Tuple, TypeVar

from rospy_util.controller.cmd import Cmd, Publishers
from rospy_util.controller.sub import Sub, Subscribers

Model = TypeVar("Model")
Msg = TypeVar("Msg")


Update = Callable[[Msg, Model], Tuple[Model, List[Cmd]]]
Subscriptions = Callable[[Model], List[Sub[Any, Msg]]]


class Controller(Generic[Model, Msg]):
    """
    A controller for communicating with ROS nodes.
    """

    def __init__(
        self,
        model: Model,
        update: Update[Msg, Model],
        subscriptions: Subscriptions[Model, Msg],
    ) -> None:
        # pylint: disable=unsubscriptable-object
        self.message_queue: Queue[Msg] = Queue()
        self.subscribers: Subscribers[Msg] = Subscribers(self.message_queue.put)
        self.publishers: Publishers = Publishers()
        self.loop_thread: Thread = Thread(target=self.__loop__, daemon=True)

        self.model = model
        self.update = update
        self.subscriptions = subscriptions

        self.loop_thread.start()

    def __loop__(self) -> None:
        while True:
            subs = self.subscriptions(self.model)
            self.subscribers.subscribe(subs)

            msg = self.message_queue.get()
            (new_model, cmds) = self.update(msg, self.model)

            for cmd in cmds:
                self.publishers.publish(cmd)

            self.model = new_model

    @staticmethod
    def run(
        model: Model,
        update: Update[Msg, Model],
        subscriptions: Subscriptions[Model, Msg],
    ) -> None:
        """
        Run a ROS controller (Remember to run rospy.init_node before and rospy.
        spin after!)
        """
        Controller(model, update, subscriptions)
