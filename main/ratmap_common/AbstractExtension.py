from abc import ABC, abstractmethod
from typing import TypedDict

from reactivex import Observable
from reactivex.subject import BehaviorSubject

from .AbstractEvent import AbstractEvent
from .EventTarget import EventTarget


class ExtensionMetadata(TypedDict):
    title: str
    description: str


class ExtensionEvent(AbstractEvent):
    pass


class AbstractExtension(EventTarget, ABC):
    __started: BehaviorSubject[bool]
    __context: object

    def __init__(self) -> None:
        super().__init__()
        self.__started = BehaviorSubject(False)
        self.__context = None

    @property
    def context(self):
        return self.__context

    @context.setter
    def context(self, context: object):
        self.__context = context

    @context.deleter
    def context(self):
        self.__context = None

    @property
    @abstractmethod
    def metadata(self) -> ExtensionMetadata:
        raise NotImplementedError()

    def start(self) -> None:
        self.__started.on_next(True)
        self.emit(ExtensionEvent("start"))

    def stop(self) -> None:
        self.__started.on_next(False)
        self.emit(ExtensionEvent("end"))

    @property
    def is_started(self) -> bool:
        return self.__started.value

    @property
    def started(self) -> Observable[bool]:
        return self.__started
