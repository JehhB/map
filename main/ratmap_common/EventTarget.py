from typing import Optional

from reactivex.abc import DisposableBase
from typing_extensions import Callable, TypeVar

from .AbstractEvent import AbstractEvent
from .CategorizedSubject import CategorizedSubject

_T = TypeVar("_T", bound=AbstractEvent, default=AbstractEvent)


class EventTarget(CategorizedSubject[_T]):
    __parent: "Optional[EventTarget[_T]]"

    def __init__(self) -> None:
        super().__init__()
        self.__parent = None

    def add_event_listener(
        self, event: str, callback: Callable[[_T], None]
    ) -> DisposableBase:
        return self.listen(event, callback)

    @property
    def parent(self):
        return self.__parent

    @parent.setter
    def parent(self, new_val: "Optional[EventTarget[_T]]"):
        self.__parent = new_val

    @parent.deleter
    def parent(self):
        self.__parent = None

    def emit(self, event: _T, target: object = None):
        propagation_stoped = False

        def stop_propagation():
            nonlocal propagation_stoped
            propagation_stoped = True

        event.stop_propagation = stop_propagation

        if target is not None:
            event.target = target

        self.broadcast(event.type, event)

        if propagation_stoped:
            return

        if self.__parent is not None:
            self.__parent.emit(event)
