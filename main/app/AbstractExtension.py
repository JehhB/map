from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List

from reactivex.subject import BehaviorSubject

from app.Container import SetterInjectable
from app.EventEmiter import EventEmitter
from app.events.DeinitEvent import DeinitEvent
from app.events.InitEvent import InitEvent


@dataclass
class ExtensionMetadata:
    name: str
    description: str
    dependencies: List[str]


class AbstractExtension(EventEmitter, SetterInjectable, ABC):
    active_subject: BehaviorSubject[bool]

    def __init__(self) -> None:
        super().__init__()
        self.active_subject = BehaviorSubject(False)

    def enable(self) -> InitEvent:
        event = InitEvent()
        self.emit_event("init", event)

        if event.is_success:
            self.active_subject.on_next(True)

        return event

    def disable(self) -> DeinitEvent:
        event = DeinitEvent()
        self.emit_event("deinit", event)

        if event.is_success:
            self.active_subject.on_next(False)

        return event

    @property
    def is_active(self):
        return self.active_subject.value

    @property
    @abstractmethod
    def metadata(self) -> ExtensionMetadata:
        raise NotImplementedError()
