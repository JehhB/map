from abc import ABC, abstractmethod
from typing import Literal, TypedDict

from reactivex import Observable
from reactivex.subject import BehaviorSubject

from ratmap_common.AbstractEvent import AbstractEvent

from .EventTarget import EventTarget


class ExtensionMetadata(TypedDict):
    title: str
    description: str


class ExtensionEvent(AbstractEvent):
    pass


class AbstractExtension(EventTarget, ABC):
    __started: BehaviorSubject[bool]

    def __init__(self) -> None:
        super().__init__()
        self.__started = BehaviorSubject(False)

    @property
    @abstractmethod
    def metadata(self) -> ExtensionMetadata:
        raise NotImplementedError()

    def start(self) -> None:
        self.__started.on_next(True)
        self.emit("start", ExtensionEvent())

    def stop(self) -> None:
        self.__started.on_next(False)
        self.emit("start", ExtensionEvent())

    @property
    def is_started(self) -> bool:
        return self.__started.value

    @property
    def started(self) -> Observable[bool]:
        return self.__started
