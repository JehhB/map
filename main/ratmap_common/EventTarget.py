from typing import Optional

from reactivex.abc import DisposableBase
from typing_extensions import Callable, TypeVar, override

from .AbstractEvent import AbstractEvent
from .CategorizedSubject import CategorizedSubject

_T = TypeVar("_T", bound=AbstractEvent, default=AbstractEvent)


class EventTarget(CategorizedSubject[_T]):
    __parent: "Optional[EventTarget[_T]]"
    __parent_disposer: Optional[DisposableBase]

    def __init__(self) -> None:
        super().__init__()
        self.__parent = None
        self.__parent_disposer = None

    def add_event_listener(
        self, event: str, callback: Callable[[_T], None]
    ) -> DisposableBase:
        return self.listen(event, callback)

    @property
    def parent(self):
        return self.__parent

    @parent.setter
    def parent(self, new_val: "Optional[EventTarget[_T]]"):
        del self.parent

        self.__parent = new_val
        self.__parent_disposer = self.subscribe(new_val)

    @parent.deleter
    def parent(self):
        if self.__parent_disposer is not None:
            self.__parent_disposer.dispose()
            self.__parent_disposer = None

        self.__parent = None

    def emit(self, event: _T, target: object = None):

        if target is not None:
            event.target = target

        event_type = event.type
        self.broadcast(event_type, event)

    @override
    def dispose(self) -> None:
        del self.parent
        return super().dispose()
