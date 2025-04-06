from reactivex.abc import DisposableBase
from typing_extensions import Callable, TypeVar

from .AbstractEvent import AbstractEvent
from .CategorizedSubject import CategorizedSubject

_T = TypeVar("_T", bound=AbstractEvent, default=AbstractEvent)


class EventTarget(CategorizedSubject[_T]):
    def __init__(self) -> None:
        super().__init__()

    def add_event_listener(
        self, event: str, callback: Callable[[_T], None]
    ) -> DisposableBase:
        return self.listen(event, callback)

    def emit(self, event: str, detail: _T):
        detail.type = event
        detail.target = self
        self.broadcast(event, detail)
