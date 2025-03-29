import tkinter as tk
from tkinter import font, messagebox
from typing import Optional, final

from app.AbstractExtension import AbstractExtension
from app.Container import Container
from app.ExtensionManager import ExtensionManager
from app.ui.ScrolledFrame import ScrolledFrame


@final
class ManageExtension(tk.Toplevel):
    _container: Container

    def __init__(
        self,
        container: Container,
        master: Optional[tk.Misc] = None,
        *args,
        **kwargs,
    ) -> None:
        super().__init__(master, *args, **kwargs, width=520, height=480)
        self._container = container

        extension_manager = self._container[ExtensionManager]
        extensions = extension_manager.items()

        self.title("Manage extensions")

        main = ScrolledFrame(self)

        for key, extension in extensions:
            ExtensionFrame(extension, main.viewPort).pack(
                padx=8, pady=8, side=tk.TOP, fill=tk.X, expand=tk.YES
            )

        main.pack(fill=tk.BOTH, expand=tk.YES)


@final
class ExtensionFrame(tk.Frame):
    def __init__(
        self,
        extension: AbstractExtension,
        master: Optional[tk.Misc] = None,
        *args,
        **kwargs,
    ) -> None:
        super().__init__(master, *args, **kwargs)
        self._extension = extension
        metadata = self._extension.metadata

        header = tk.Frame(self)
        default_font = font.nametofont("TkTextFont")

        default_font_conf = (
            default_font.actual("family"),
            default_font.actual("size"),
            default_font.actual("weight"),
        )

        self._enable_button = tk.Button(header, text="Enable", command=self._enable)
        self._disable_button = tk.Button(header, text="Disable", command=self._disable)

        tk.Label(
            header,
            text=metadata.name,
            font=(default_font_conf[0], default_font_conf[1], "bold"),
        ).pack(side=tk.LEFT)

        header.pack(fill=tk.X, expand=tk.YES, pady=4)

        description = tk.Label(self, text=metadata.description, justify=tk.LEFT)
        _ = description.bind(
            "<Configure>",
            lambda e: description.config(wraplength=description.winfo_width()),
        )
        description.pack(fill=tk.X, expand=tk.YES, pady=4)

        _ = self._extension.active_subject.subscribe(on_next=self._render_button)

    def _render_button(self, is_active: bool):
        if is_active:
            self._enable_button.pack_forget()
            self._disable_button.pack(side=tk.LEFT, padx=8)
        else:
            self._disable_button.pack_forget()
            self._enable_button.pack(side=tk.LEFT, padx=8)

    def _enable(self):
        init_event = self._extension.enable()
        if not init_event.is_success:
            messagebox.showerror("Error starting extension", str(init_event.detail))

    def _disable(self):
        deinit_event = self._extension.disable()
        if not deinit_event.is_success:
            messagebox.showerror(
                "Error terminating extension", str(deinit_event.detail)
            )
