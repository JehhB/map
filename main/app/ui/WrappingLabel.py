import tkinter as tk


class WrappingLabel(tk.Label):
    def __init__(
        self,
        *args,
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)
        _ = self.bind(
            "<Configure>",
            lambda e: self.config(wraplength=self.winfo_width()),
        )
