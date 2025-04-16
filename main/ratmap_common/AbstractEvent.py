from abc import ABC
from typing import Any

from typing_extensions import override


class AbstractEvent(ABC):
    type: str
    target: object
    detail: Any

    def __init__(self, type: str, target: object = None, detail: Any = None):
        self.type = type
        self.target = target
        self.detail = detail if detail is not None else {}

    @override
    def __repr__(self) -> str:
        return f"<{self.__class__.__name__} target={repr(self.target)}>"
