from __future__ import annotations

import tkinter as tk
from typing import Callable, Optional, Set, TypeVar, Union, overload

from reactivex import Observable, Subject, operators
from reactivex.abc import DisposableBase
from typing_extensions import override

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


@overload
def sync_observable_to_variable(
    observable: Observable[str], var: tk.StringVar, master: Optional[tk.Misc]
) -> DisposableBase: ...


@overload
def sync_observable_to_variable(
    observable: Observable[float], var: tk.DoubleVar, master: Optional[tk.Misc]
) -> DisposableBase: ...


@overload
def sync_observable_to_variable(
    observable: Observable[int], var: tk.IntVar, master: Optional[tk.Misc]
) -> DisposableBase: ...


@overload
def sync_observable_to_variable(
    observable: Observable[bool], var: tk.BooleanVar, master: Optional[tk.Misc]
) -> DisposableBase: ...


def sync_observable_to_variable(
    observable: Union[
        Observable[str], Observable[float], Observable[int], Observable[bool]
    ],
    var: Union[tk.StringVar, tk.DoubleVar, tk.IntVar, tk.BooleanVar],
    master: Optional[tk.Misc] = None,
) -> DisposableBase:
    def update(x: Union[str, float, int]):
        if x != var.get():
            var.set(x)  # type: ignore # pyright: ignore[reportArgumentType]

    return observable.pipe(operators.throttle_first(0.1)).subscribe(
        safe_callback(master, update)
    )


@overload
def bind_subject_to_variable(
    subject: Subject[str],
    var: tk.StringVar,
    master: Optional[tk.Misc],
    on_change: Optional[Callable[[str], None]] = None,
) -> DisposableBase: ...


@overload
def bind_subject_to_variable(
    subject: Subject[float],
    var: tk.DoubleVar,
    master: Optional[tk.Misc],
    on_change: Optional[Callable[[float], None]] = None,
) -> DisposableBase: ...


@overload
def bind_subject_to_variable(
    subject: Subject[int],
    var: tk.IntVar,
    master: Optional[tk.Misc],
    on_change: Optional[Callable[[int], None]] = None,
) -> DisposableBase: ...


@overload
def bind_subject_to_variable(
    subject: Subject[bool],
    var: tk.BooleanVar,
    master: Optional[tk.Misc],
    on_change: Optional[Callable[[bool], None]] = None,
) -> DisposableBase: ...


def bind_subject_to_variable(
    subject: Union[Subject[str], Subject[float], Subject[int], Subject[bool]],
    var: Union[tk.StringVar, tk.DoubleVar, tk.IntVar, tk.BooleanVar],
    master: Optional[tk.Misc] = None,
    on_change: Optional[
        Union[
            Callable[[str], None],
            Callable[[float], None],
            Callable[[int], None],
            Callable[[bool], None],
        ]
    ] = None,
) -> DisposableBase:

    def _on_change(_a: str, _b: str, _c: str) -> object:
        new_val = var.get()
        subject.on_next(new_val)  # type: ignore # pyright: ignore[reportArgumentType]

        if on_change is not None:
            on_change(new_val)  # type: ignore # pyright: ignore[reportArgumentType]

        return None

    callback_name = var.trace_add("write", _on_change)

    subject_disposer = sync_observable_to_variable(subject, var, master)  # type: ignore # pyright: ignore
    callback_disposer = CallbackDisposer(callback_name, var)

    disposer = SetDisposer()
    disposer.add(subject_disposer)
    disposer.add(callback_disposer)

    return disposer


class __DisposableBind(DisposableBase):
    def __init__(self, widget: tk.Misc, seq: str, cbn: str) -> None:
        self.__widget = widget
        self.__cbn = cbn
        self.__seq = seq

    @override
    def dispose(self) -> None:
        self.__widget.unbind(self.__seq, self.__cbn)


def disposable_bind(
    widget: tk.Misc, seq: str, func: Callable[[tk.Event[tk.Misc]], object]
):
    cbn = widget.bind(seq, func)
    return __DisposableBind(widget, seq, cbn)


class CallbackDisposer(DisposableBase):
    def __init__(self, cbn: str, variable: tk.Variable) -> None:
        super().__init__()
        self.__cbn = cbn
        self.__variable = variable

    @override
    def dispose(self) -> None:
        self.__variable.trace_remove("write", self.__cbn)


class SetDisposer(DisposableBase):
    __disposers: Set[DisposableBase]

    def __init__(self) -> None:
        super().__init__()
        self.__disposers = set()

    @override
    def dispose(self) -> None:
        for disposer in self.__disposers:
            disposer.dispose()
        self.__disposers = set()

    def add(self, *disposables: DisposableBase):
        for d in disposables:
            self.__disposers.add(d)


def safe_dispose(disposable: Optional[DisposableBase]):
    if disposable is not None:
        disposable.dispose()
