from __future__ import annotations

import tkinter
from tkinter.font import Font

from reactivex import Subject
from typing_extensions import (
    TYPE_CHECKING,
    Any,
    Callable,
    List,
    Literal,
    Optional,
    Protocol,
    Tuple,
    TypeAlias,
    TypedDict,
    Union,
    Unpack,
)

if TYPE_CHECKING:
    from cv2.typing import MatLike
    from PIL.Image import Image as PilImage
    from reactivex import Observable


ButtonCommand: TypeAlias = Union[str, Callable[[], Any]]
TtkCompound: TypeAlias = Literal[
    "", "text", "image", "top", "left", "center", "right", "bottom", "none"
]
Compound: TypeAlias = Literal["top", "left", "center", "right", "bottom", "none"]
Cursor: TypeAlias = Union[
    str, Tuple[str], Tuple[str, str], Tuple[str, str, str], Tuple[str, str, str, str]
]
Anchor: TypeAlias = Literal["nw", "n", "ne", "w", "center", "e", "sw", "s", "se"]
ScreenUnits: TypeAlias = Union[str, float]
FontDescription: TypeAlias = Union[
    str,
    Font,
    List[Any],
    Tuple[str],
    Tuple[str, int, Unpack[Tuple[str, ...]]],
    Tuple[str, int, Union[List[str], Tuple[str, ...]]],
]
Relief: TypeAlias = Literal["raised", "sunken", "flat", "ridge", "solid", "groove"]
EntryValidateCommand: TypeAlias = Union[
    str, List[str], Tuple[str, ...], Callable[[], bool]
]
XYScrollCommand: TypeAlias = Union[str, Callable[[float, float], object]]
TakeFocusValue: TypeAlias = Union[
    bool, Callable[[str], Optional[bool]], Literal[0, 1, ""]
]
ImageSpec: TypeAlias = Union[str, "Image"]


class Image(Protocol):
    def width(self) -> int: ...
    def height(self) -> int: ...


class BaseButtonKwargs(TypedDict, total=False):
    class_: str
    compound: TtkCompound
    cursor: Cursor
    default: Literal["normal", "active", "disabled"]
    name: str
    state: Literal["normal", "active", "disabled"]
    style: str
    takefocus: TakeFocusValue
    underline: int
    padding: int
    width: Union[int, Literal[""]]
    text: Union[float, str]


class ButtonKwargs(BaseButtonKwargs, total=False):
    command: Callable[[], None]
    textvariable: tkinter.StringVar
    textobservable: "Observable[str]"
    stateobservable: "Observable[Literal['normal', 'active', 'disabled']]"


class BaseLabelKwargs(TypedDict, total=False):
    anchor: Anchor
    background: str
    border: ScreenUnits
    borderwidth: ScreenUnits
    class_: str
    compound: TtkCompound
    cursor: Cursor
    font: FontDescription
    foreground: str
    justify: Literal["left", "center", "right"]
    name: str
    padding: int
    relief: Relief
    state: Literal["normal", "active", "disabled"]
    style: str
    takefocus: TakeFocusValue
    underline: int
    width: Union[int, Literal[""]]
    wraplength: ScreenUnits
    text: Union[float, str]


class LabelKwargs(BaseLabelKwargs, total=False):
    textvariable: tkinter.StringVar
    textobservable: "Observable[str]"


class BaseImageLabelKwargs(TypedDict, total=False):
    activebackground: str
    activeforeground: str
    anchor: Anchor
    background: str
    bd: ScreenUnits
    bg: str
    border: ScreenUnits
    borderwidth: ScreenUnits
    compound: Compound
    cursor: Cursor
    disabledforeground: str
    fg: str
    font: FontDescription
    foreground: str
    highlightbackground: str
    highlightcolor: str
    highlightthickness: ScreenUnits
    justify: Literal["left", "center", "right"]
    name: str
    padx: ScreenUnits
    pady: ScreenUnits
    relief: Relief
    state: Literal["normal", "active", "disabled"]
    takefocus: TakeFocusValue
    underline: int
    wraplength: ScreenUnits


class ImageLabelKwargs(BaseImageLabelKwargs, total=False):
    width: int
    height: int
    imageobservable: "Observable[Union[PilImage, MatLike, str, None]]"


class BaseEntryKwargs(TypedDict, total=False):
    background: str
    class_: str
    cursor: Cursor
    exportselection: bool
    font: FontDescription
    foreground: str
    invalidcommand: EntryValidateCommand
    justify: Literal["left", "center", "right"]
    name: str
    show: str
    state: Literal["normal", "disabled", "readonly"]
    style: str
    takefocus: TakeFocusValue
    validate: Literal["none", "focus", "focusin", "focusout", "key", "all"]
    validatecommand: EntryValidateCommand
    width: int
    xscrollcommand: XYScrollCommand


class EntryKwargs(BaseEntryKwargs, total=False):
    textvariable: tkinter.StringVar
    textsubject: "Subject[str]"
    stateobservable: "Observable[Literal['normal', 'disabled', 'readonly']]"


class BaseSpinboxKwargs(TypedDict, total=False):
    background: str
    class_: str
    command: Union[Callable[[], object], str, list[str], tuple[str, ...]]
    cursor: Cursor
    exportselection: bool
    font: FontDescription
    foreground: str
    format: str
    from_: float
    increment: float
    invalidcommand: EntryValidateCommand
    justify: Literal["left", "center", "right"]
    name: str
    state: Literal["normal", "disabled", "readonly"]
    style: str
    takefocus: TakeFocusValue
    to: float
    validate: Literal["none", "focus", "focusin", "focusout", "key", "all"]
    validatecommand: EntryValidateCommand
    width: int
    wrap: bool
    xscrollcommand: XYScrollCommand


class SpinboxKwargs(BaseSpinboxKwargs, total=False):
    textvariable: tkinter.DoubleVar
    value_subject: Subject[float]
    state_observable: Observable[Literal["normal", "disabled", "readonly"]]


class BaseScaleKwargs(TypedDict, total=False):
    class_: str
    command: Union[str, Callable[[str], object]]
    cursor: Cursor
    from_: float
    length: ScreenUnits
    name: str
    orient: Literal["horizontal", "vertical"]
    state: str
    style: str
    takefocus: TakeFocusValue
    to: float
    value: float


class ScaleKwargs(BaseScaleKwargs, total=False):
    variable: tkinter.DoubleVar
    value_subject: Subject[float]
    state_observable: Observable[Literal["normal", "disabled"]]


class SubMenuKwargs(TypedDict, total=False):
    activebackground: str
    activeborderwidth: ScreenUnits
    activeforeground: str
    background: str
    bd: ScreenUnits
    bg: str
    border: ScreenUnits
    borderwidth: ScreenUnits
    cursor: Cursor
    disabledforeground: str
    fg: str
    font: FontDescription
    foreground: str
    name: str
    postcommand: Union[Callable[[], object], str]
    relief: Relief
    selectcolor: str
    takefocus: TakeFocusValue
    tearoffcommand: Union[Callable[[str, str], object], str]
    title: str
    type: Literal["menubar", "tearoff", "normal"]


class MenuKwargs(SubMenuKwargs, total=False):
    tearoff: Union[bool, Literal[0, 1]]


class FrameKwargs(TypedDict, total=False):
    border: ScreenUnits
    borderwidth: ScreenUnits
    class_: str
    cursor: Cursor
    height: ScreenUnits
    name: str
    relief: Relief
    style: str
    takefocus: TakeFocusValue
    width: ScreenUnits


__all__ = [
    "Anchor",
    "BaseButtonKwargs",
    "BaseEntryKwargs",
    "BaseImageLabelKwargs",
    "BaseLabelKwargs",
    "BaseScaleKwargs",
    "BaseSpinboxKwargs",
    "ButtonCommand",
    "ButtonKwargs",
    "Compound",
    "Cursor",
    "EntryKwargs",
    "EntryValidateCommand",
    "FontDescription",
    "Image",
    "ImageLabelKwargs",
    "ImageSpec",
    "LabelKwargs",
    "MenuKwargs",
    "Relief",
    "ScaleKwargs",
    "ScreenUnits",
    "SpinboxKwargs",
    "SubMenuKwargs",
    "TakeFocusValue",
    "TtkCompound",
    "XYScrollCommand",
    "FrameKwargs",
]
