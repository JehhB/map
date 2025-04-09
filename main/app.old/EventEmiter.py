from typing import Callable, Dict, Set

from app.events.AbstractEvent import AbstractEvent

EventHandler = Callable[[AbstractEvent], None]


class EventEmitter:
    _event_handlers: Dict[str, Set[EventHandler]]

    def __init__(self) -> None:
        self._event_handlers = dict()

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
