from abc import ABC
from typing import Any


class AbstractEvent(ABC):
    type: str
    target: object
    detail: Any
