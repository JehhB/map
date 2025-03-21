from app.ui.Main import Main
from app.ui.MenuBar import MenuBar


class Application:
    def __init__(self) -> None:
        self.main_window = Main()
        self.menu_bar = MenuBar()
        self.main_window.config(menu=self.menu_bar)

    def mainloop(self):
        self.main_window.mainloop()


application = Application()
