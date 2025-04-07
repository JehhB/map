from abc import ABC
from typing import Any, Callable


class AbstractEvent(ABC):
    type: str
    target: object
    detail: Any

    stop_propagation: Callable[[], None]

    def __init__(self, type: str, target: object = None, detail: Any = None):
        self.type = type
        self.target = target
        self.detail = detail if detail is not None else {}

        self.stop_propagation = lambda: None
