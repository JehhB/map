from abc import ABC


class AbstractEvent(ABC):
    is_success: bool = True
