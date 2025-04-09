import tkinter as tk
from typing import Any, Callable, Optional, TypeVar

from reactivex import Observable, Subject
from reactivex.abc import DisposableBase

_T = TypeVar("_T")


def safe_callback(
    master: Optional[tk.Misc], callable: Callable[[_T], object]
) -> Callable[[_T], None]:
    def func(x: _T):
        if master is not None:
            _ = master.after(0, callable, x)
        else:
            _ = callable(x)

    return func


_Te = TypeVar(
    "_Te",
    tk.Entry,
    tk.Spinbox,
)


def sync_obserable_to_variable(
    observable: Observable[_T],
    var: tk.Variable,
    master: Optional[tk.Misc] = None,
) -> DisposableBase:
    def update(x: _T):
        if x != var.get():
            var.set(x)

    return observable.subscribe(safe_callback(master, update))


def bind_subject_to_widget(
    widget: _Te,
    subject: Subject[str],
    var: Optional[tk.StringVar] = None,
) -> DisposableBase:
    _var = tk.StringVar() if var is None else var

    def on_change(_a: str, _b: str, _c: str) -> object:
        subject.on_next(_var.get())
        return None

    _ = widget.config(textvariable=_var)
    _ = _var.trace_add("write", on_change)

    return sync_obserable_to_variable(subject, _var, widget)


def safe_parse_int(value: Any, default: int = 0):
    try:
        return int(value)
    except (ValueError, TypeError):
        return default


def safe_parse_float(value: Any, default: float = 0.0):
    try:
        return float(value)
    except (ValueError, TypeError):
        return default
