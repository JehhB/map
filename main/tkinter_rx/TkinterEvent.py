from __future__ import annotations

import tkinter as tk
from typing import Any, Optional, Union

from typing_extensions import Generic, TypeVar

from ratmap_common.BaseEvent import BaseEvent

_T = TypeVar("_T", bound=tk.Misc, default=tk.Misc)


class TkinterEventDetail(
    tk.Event, Generic[_T]  # pyright: ignore[reportMissingTypeArgument]
):
    additional: Any
    widget: _T

    def __init__(self, additional: Any = None) -> None:
        super().__init__()
        self.type = tk.EventType.VirtualEvent
        self.additional = additional


_sentinel = TkinterEventDetail()


class TkinterEvent(BaseEvent):
    detail: Union[tk.Event[tk.Misc], Exception, None]

    def __init__(
        self,
        type: str,
        target: Optional[tk.Misc] = None,
        detail: Union[tk.Event[tk.Misc], Exception, None] = _sentinel,
    ):
        if detail is _sentinel:
            detail = TkinterEventDetail()

        if isinstance(detail, tk.Event) and target is not None:
            detail.widget = target

        super().__init__(type, target, detail)


__all__ = ["TkinterEvent", "TkinterEventDetail"]
