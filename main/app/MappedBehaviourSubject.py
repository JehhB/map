from typing import Callable, Generic, TypeVar

from reactivex.abc import DisposableBase
from reactivex.subject import BehaviorSubject
from typing_extensions import override

_T = TypeVar("_T")
_U = TypeVar("_U")


class MappedBehaviourSubject(Generic[_T, _U], BehaviorSubject[_T], DisposableBase):
    _disposer: DisposableBase
    _subject: BehaviorSubject[_U]
    _map_in: Callable[[_T, _U], _U]

    def __init__(
        self,
        value: _T,
        subject: BehaviorSubject[_U],
        map_out: Callable[[_U], _T],
        map_in: Callable[[_T, _U], _U],
    ) -> None:
        super().__init__(value)

        self._disposer = subject.subscribe(
            on_next=lambda x: super(BehaviorSubject, self).on_next(map_out(x)),
            on_error=self.on_error,
            on_completed=self.on_completed,
        )
        self._map_in = map_in
        self._subject = subject

    @override
    def on_next(self, value: _T) -> None:
        self._subject.on_next(self._map_in(value, self._subject.value))

    @override
    def dispose(self) -> None:
        self._disposer.dispose()
