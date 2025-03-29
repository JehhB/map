from abc import ABC
from typing import Any


class AbstractEvent(ABC):
    is_success: bool = True
    detail: Any = None
