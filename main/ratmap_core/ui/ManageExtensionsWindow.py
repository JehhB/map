import tkinter as tk
from typing import Optional

from tkinter_rx import ScrolledFrame

from ..ExtensionManager import ExtensionManager
from .ExtensionFrame import ExtensionFrame


class ManageExtensionsWindow(tk.Toplevel):
    def __init__(
        self,
        extension_manager: ExtensionManager,
        master: Optional[tk.Misc] = None,
    ) -> None:
        super().__init__(
            master,
        )
        self.title("Manage Extensions")

        main = ScrolledFrame(self)

        extensions = extension_manager.items()

        for key, extension in extensions:
            ExtensionFrame(extension, main.viewPort).pack(
                padx=8, pady=8, side=tk.TOP, fill=tk.X, expand=tk.YES
            )

        main.pack(fill=tk.BOTH, expand=tk.YES)
