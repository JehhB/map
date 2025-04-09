import platform
import tkinter as tk
from typing import Any, Optional, final


@final
class ScrolledFrame(tk.Frame):
    def __init__(self, parent: tk.Misc):
        super().__init__(parent)

        bg = parent.cget("bg")
        self.canvas = tk.Canvas(self, borderwidth=0, background=bg)

        self.viewPort = tk.Frame(self.canvas, background=bg)
        self.vsb = tk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
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

    def onFrameConfigure(self, _event: Any):
        _ = self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def onCanvasConfigure(self, event: Any):
        canvas_width = event.width
        _ = self.canvas.itemconfig(self.canvas_window, width=canvas_width)

    def onMouseWheel(self, event: Any):
        if platform.system() == "Windows":
            self.canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        elif platform.system() == "Darwin":
            self.canvas.yview_scroll(int(-1 * event.delta), "units")
        else:
            if event.num == 4:
                self.canvas.yview_scroll(-1, "units")
            elif event.num == 5:
                self.canvas.yview_scroll(1, "units")

    def onEnter(self, event: Any):
        if platform.system() == "Linux":
            self.canvas.bind_all("<Button-4>", self.onMouseWheel)
            self.canvas.bind_all("<Button-5>", self.onMouseWheel)
        else:
            self.canvas.bind_all("<MouseWheel>", self.onMouseWheel)

    def onLeave(self, event: Any):
        if platform.system() == "Linux":
            self.canvas.unbind_all("<Button-4>")
            self.canvas.unbind_all("<Button-5>")
        else:
            self.canvas.unbind_all("<MouseWheel>")
