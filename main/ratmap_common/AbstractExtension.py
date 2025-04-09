from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TypedDict

from reactivex import Observable
from reactivex.subject import BehaviorSubject

from .AbstractEvent import AbstractEvent
from .EventTarget import EventTarget


class ExtensionMetadata(TypedDict):
    id: str
    title: str
    description: str


@dataclass
class ExtensionEventDetail:
    id: str


class ExtensionEvent(AbstractEvent):
    def __init__(self, id: str, type: str, target: object = None):
        super().__init__(type, target, ExtensionEventDetail(id))


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
        id = self.metadata["id"]
        self.__started.on_next(True)
        self.emit(ExtensionEvent(id, "start"))
        self.emit(ExtensionEvent(id, "start." + id))

    def stop(self) -> None:
        id = self.metadata["id"]
        self.__started.on_next(False)
        self.emit(ExtensionEvent(id, "end"))
        self.emit(ExtensionEvent(id, "end." + id))

    @property
    def is_started(self) -> bool:
        return self.__started.value

    @property
    def started(self) -> Observable[bool]:
        return self.__started
