from typing import Callable, List, Tuple, TypeVar

import reactivex.operators as ops
from reactivex import Subject
from reactivex.abc import DisposableBase
from typing_extensions import override

_T = TypeVar("_T")


class CategorizedSubject(Subject[Tuple[str, _T]]):
    __disposers: List[DisposableBase]

    def __init__(self) -> None:
        super().__init__()
        self.__disposers = list()

    def listen(self, category: str, callback: Callable[[_T], None]) -> DisposableBase:
        disposer = self.pipe(
            ops.filter(lambda pair: pair[0].startswith(category)),
            ops.map(lambda pair: pair[1]),
        ).subscribe(on_next=callback)
        self.__disposers.append(disposer)
        return disposer

    def broadcast(self, category: str, data: _T):
        self.on_next((category, data))

    @override
    def dispose(self) -> None:
        for disposer in self.__disposers:
            disposer.dispose()
        self.__disposers = list()

        return super().dispose()
