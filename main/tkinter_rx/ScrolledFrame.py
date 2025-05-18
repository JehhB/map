from __future__ import annotations

import platform
import tkinter as tk
from tkinter import ttk
from typing import Any, Optional

from typing_extensions import Unpack

from .typing import FrameKwargs


class ScrolledFrame(tk.Frame):
    canvas: tk.Canvas
    viewPort: ttk.Frame
    vsb: ttk.Scrollbar
    canvas_window: int

    def __init__(self, parent: tk.Misc, **kwargs: Unpack[FrameKwargs]):
        super().__init__(parent, **kwargs)

        bg: str = parent.cget("bg")
        self.canvas = tk.Canvas(self, borderwidth=0, background=bg)

        self.viewPort = ttk.Frame(self.canvas)
        self.vsb = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        _ = self.canvas.configure(yscrollcommand=self.vsb.set)

        self.vsb.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)
        self.canvas_window = self.canvas.create_window(
            (4, 4),
            window=self.viewPort,
            anchor="nw",
            tags="self.viewPort",
        )

        _ = self.viewPort.bind("<Configure>", self.onFrameConfigure)
        _ = self.canvas.bind("<Configure>", self.onCanvasConfigure)

        _ = self.viewPort.bind("<Enter>", self.onEnter)
        _ = self.viewPort.bind("<Leave>", self.onLeave)

        self.onFrameConfigure(None)

    def onFrameConfigure(self, _event: Optional[tk.Event[ttk.Frame]]):
        _ = self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def onCanvasConfigure(self, event: tk.Event[tk.Canvas]):
        canvas_width = event.width
        _ = self.canvas.itemconfig(self.canvas_window, width=canvas_width)

    def onMouseWheel(self, event: tk.Event[tk.Misc]):
        if platform.system() == "Windows":
            self.canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        elif platform.system() == "Darwin":
            self.canvas.yview_scroll(int(-1 * event.delta), "units")
        else:
            if event.num == 4:
                self.canvas.yview_scroll(-1, "units")
            elif event.num == 5:
                self.canvas.yview_scroll(1, "units")

    def onEnter(self, _event: tk.Event[ttk.Frame]):
        if platform.system() == "Linux":
            _ = self.canvas.bind_all("<Button-4>", self.onMouseWheel)
            _ = self.canvas.bind_all("<Button-5>", self.onMouseWheel)
        else:
            _ = self.canvas.bind_all("<MouseWheel>", self.onMouseWheel)

    def onLeave(self, _event: tk.Event[ttk.Frame]):
        if platform.system() == "Linux":
            self.canvas.unbind_all("<Button-4>")
            self.canvas.unbind_all("<Button-5>")
        else:
            self.canvas.unbind_all("<MouseWheel>")
