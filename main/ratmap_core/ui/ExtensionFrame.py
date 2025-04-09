import tkinter as tk
import traceback
from tkinter import font, messagebox
from typing import Optional, Set, final

from reactivex.abc import DisposableBase
from typing_extensions import Unpack, override

from ratmap_common import AbstractExtension
from tkinter_rx import Button, Frame, WrappingLabel
from tkinter_rx.typing import FrameKwargs


@final
class ExtensionFrame(Frame):
    def __init__(
        self,
        extension: AbstractExtension,
        master: Optional[tk.Misc] = None,
        **kwargs: Unpack[FrameKwargs],
    ) -> None:
        super().__init__(master, **kwargs)

        self._extension = extension
        metadata = self._extension.metadata

        header = tk.Frame(self)
        default_font = font.nametofont("TkTextFont")

        default_font_conf = (
            default_font.actual("family"),
            default_font.actual("size"),
            default_font.actual("weight"),
        )

        self._enable_button = Button(header, text="Enable", command=self._enable)
        self._disable_button = Button(header, text="Disable", command=self._disable)

        tk.Label(
            header,
            text=metadata["title"],
            font=(default_font_conf[0], default_font_conf[1], "bold"),
        ).pack(side=tk.LEFT)

        header.pack(fill=tk.X, expand=tk.YES, pady=4)

        WrappingLabel(self, text=metadata["description"], justify=tk.LEFT).pack(
            side=tk.TOP, fill=tk.X, expand=tk.YES, pady=4
        )

        self.__disposer: Set[DisposableBase] = set()

        self.__disposer.add(
            self._extension.started.subscribe(on_next=self._render_button)
        )

    def _render_button(self, is_active: bool):
        if is_active:
            self._enable_button.pack_forget()
            self._disable_button.pack(side=tk.LEFT, padx=8)
        else:
            self._disable_button.pack_forget()
            self._enable_button.pack(side=tk.LEFT, padx=8)

    def _enable(self):
        try:
            print(self._extension)
            self._extension.start()
        except Exception as e:
            traceback.print_exc()
            _ = messagebox.showerror("Error starting extension", str(e))

    def _disable(self):
        try:
            self._extension.stop()
        except Exception as e:
            traceback.print_exc()
            _ = messagebox.showerror("Error terminating extension", str(e))

    @override
    def destroy(self) -> None:
        for disposer in self.__disposer:
            disposer.dispose()
        self.__disposer = set()

        return super().destroy()
