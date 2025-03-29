from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable, Dict, List, Set

from reactivex.subject import BehaviorSubject

from app.Container import SetterInjectable
from app.events.AbstractEvent import AbstractEvent
from app.events.DeinitEvent import DeinitEvent
from app.events.InitEvent import InitEvent

EventHandler = Callable[[AbstractEvent], None]


@dataclass
class ExtensionMetadata:
    name: str
    description: str
    dependencies: List[str]


class AbstractExtension(SetterInjectable, ABC):
    active_subject: BehaviorSubject[bool]
    _event_handlers: Dict[str, Set[EventHandler]]

    def __init__(self) -> None:
        super().__init__()

        self._event_handlers = dict()
        self.active_subject = BehaviorSubject(False)

    def add_event_handler(self, event: str, event_handler: EventHandler):
        if event not in self._event_handlers:
            self._event_handlers[event] = set()
        self._event_handlers[event].add(event_handler)

    def remove_event_handler(self, event: str, event_handler: EventHandler) -> bool:
        if (
            event not in self._event_handlers
            or event_handler not in self._event_handlers[event]
        ):
            return False
        self._event_handlers[event].remove(event_handler)

        if len(self._event_handlers) == 0:
            del self._event_handlers[event]

        return True

    def emit_event(self, event: str, eventObject: AbstractEvent):
        if event not in self._event_handlers:
            return

        for handler in self._event_handlers[event]:
            handler(eventObject)

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
        self.active_subject.value

    @property
    @abstractmethod
    def metadata(self) -> ExtensionMetadata:
        raise NotImplementedError()
