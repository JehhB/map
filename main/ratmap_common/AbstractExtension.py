from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, TypedDict

from reactivex import Observable
from reactivex.subject import BehaviorSubject
from typing_extensions import NotRequired

from .BaseEvent import BaseEvent
from .EventTarget import EventTarget


class ExtensionMetadata(TypedDict):
    id: str
    title: str
    description: str
    dependency: NotRequired[List[str]]


@dataclass
class ExtensionEventDetail:
    id: str


class ExtensionEvent(BaseEvent):
    def __init__(self, id: str, type: str, target: object = None):
        super().__init__(type, target, ExtensionEventDetail(id))


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
        id = self.metadata["id"]
        self.__started.on_next(True)
        self.emit(ExtensionEvent(id, "extension.start." + id))

    def stop(self) -> None:
        id = self.metadata["id"]
        self.__started.on_next(False)
        self.emit(ExtensionEvent(id, "extension.stop." + id))

    @property
    def is_started(self) -> bool:
        return self.__started.value

    @property
    def started(self) -> Observable[bool]:
        return self.__started
