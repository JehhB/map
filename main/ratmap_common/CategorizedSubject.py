from typing import Callable, List, Tuple, TypeVar

import reactivex.operators as ops
from reactivex import Observable, Subject, observable
from reactivex.abc import DisposableBase
from typing_extensions import override

_T = TypeVar("_T")


class CategorizedSubject(Subject[Tuple[str, _T]]):
    __disposers: List[DisposableBase]

    def __init__(self) -> None:
        super().__init__()
        self.__disposers = list()

    def listen(self, category: str, callback: Callable[[_T], None]) -> DisposableBase:
        observable = self.observe(category)
        disposer = observable.subscribe(on_next=callback)

        self.__disposers.append(disposer)
        return disposer

    def observe(self, category: str) -> Observable[_T]:
        return self.pipe(
            ops.filter(lambda pair: pair[0].startswith(category)),
            ops.map(lambda pair: pair[1]),
        )

    def broadcast(self, category: str, data: _T):
        self.on_next((category, data))

    @override
    def dispose(self) -> None:
        for disposer in self.__disposers:
            disposer.dispose()
        self.__disposers = list()

        return super().dispose()
