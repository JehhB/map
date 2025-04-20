import tkinter as tk
from typing import Callable, Optional, Set, Tuple, TypeVar, Union, overload

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


def sync_observable_to_variable(
    observable: Union[Observable[str], Observable[float], Observable[int]],
    var: Union[tk.StringVar, tk.DoubleVar, tk.IntVar],
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
    on_change: Optional[Callable[[str], None]],
) -> Tuple[DisposableBase, str]: ...


@overload
def bind_subject_to_variable(
    subject: Subject[float],
    var: tk.DoubleVar,
    master: Optional[tk.Misc],
    on_change: Optional[Callable[[float], None]],
) -> Tuple[DisposableBase, str]: ...


@overload
def bind_subject_to_variable(
    subject: Subject[int],
    var: tk.IntVar,
    master: Optional[tk.Misc],
    on_change: Optional[Callable[[int], None]],
) -> Tuple[DisposableBase, str]: ...


def bind_subject_to_variable(
    subject: Union[Subject[str], Subject[float], Subject[int]],
    var: Union[tk.StringVar, tk.DoubleVar, tk.IntVar],
    master: Optional[tk.Misc] = None,
    on_change: Optional[
        Union[Callable[[str], None], Callable[[float], None], Callable[[int], None]]
    ] = None,
) -> Tuple[DisposableBase, str]:

    def _on_change(_a: str, _b: str, _c: str) -> object:
        new_val = var.get()
        subject.on_next(new_val)  # type: ignore # pyright: ignore[reportArgumentType]

        if on_change is not None:
            on_change(new_val)  # type: ignore # pyright: ignore[reportArgumentType]

        return None

    callback_name = var.trace_add("write", _on_change)
    disposer = sync_observable_to_variable(subject, var, master)  # type: ignore # pyright: ignore

    return (disposer, callback_name)


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

    def add(self, disposable: DisposableBase):
        self.__disposers.add(disposable)


def safe_dispose(disposable: Optional[DisposableBase]):
    if disposable is not None:
        disposable.dispose()
