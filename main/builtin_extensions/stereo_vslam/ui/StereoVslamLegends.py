import tkinter as tk
from typing import Optional

from tkinter_rx import Frame, Label


class StereoVslamLegends(Frame):
    def __init__(self, master: Optional[tk.Misc]):
        super().__init__(master)

        legend = Frame(self)
        canvas = tk.Canvas(legend, width=8, height=8, bg="#000000")
        canvas.pack(side=tk.LEFT, expand=tk.NO)
        Label(legend, text="Unknown space").pack(side=tk.LEFT, expand=tk.NO, padx=2)
        legend.pack(side=tk.TOP, expand=tk.YES, fill=tk.X, padx=4)

        legend = Frame(self)
        canvas = tk.Canvas(legend, width=8, height=8, bg="#eeeeee")
        canvas.pack(side=tk.LEFT, expand=tk.NO)
        Label(legend, text="Ground").pack(side=tk.LEFT, expand=tk.NO, padx=2)
        legend.pack(side=tk.TOP, expand=tk.YES, fill=tk.X, padx=4)

        legend = Frame(self)
        canvas = tk.Canvas(legend, width=8, height=8, bg="#dd2222")
        canvas.pack(side=tk.LEFT, expand=tk.NO)
        Label(legend, text="Obstacle").pack(side=tk.LEFT, expand=tk.NO, padx=2)
        legend.pack(side=tk.TOP, expand=tk.YES, fill=tk.X, padx=4)

        legend = Frame(self)
        canvas = tk.Canvas(legend, width=18, height=3, bg="#ffff00")
        canvas.pack(side=tk.LEFT, expand=tk.NO)
        Label(legend, text="Current position").pack(side=tk.LEFT, expand=tk.NO, padx=2)
        legend.pack(side=tk.TOP, expand=tk.YES, fill=tk.X, padx=4)

        legend = Frame(self)
        canvas = tk.Canvas(legend, width=18, height=3, bg="#0000ff")
        canvas.pack(side=tk.LEFT, expand=tk.NO)
        Label(legend, text="Recorded path").pack(side=tk.LEFT, expand=tk.NO, padx=2)
        legend.pack(side=tk.TOP, expand=tk.YES, fill=tk.X, padx=4)

        legend = Frame(self)
        canvas = tk.Canvas(legend, width=18, height=3, bg="#00ff00")
        canvas.pack(side=tk.LEFT, expand=tk.NO)
        Label(legend, text="Planned path").pack(side=tk.LEFT, expand=tk.NO, padx=2)
        legend.pack(side=tk.TOP, expand=tk.YES, fill=tk.X, padx=4)

        legend = Frame(self)
        canvas = tk.Canvas(legend, width=16, height=16)
        _ = canvas.create_oval(1, 2, 15, 14, fill="#22dd22", outline="")
        canvas.pack(side=tk.LEFT, expand=tk.NO)
        Label(legend, text="Target").pack(side=tk.LEFT, expand=tk.NO, padx=2)
        legend.pack(side=tk.TOP, expand=tk.YES, fill=tk.X, padx=4)
