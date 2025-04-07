import os

from ratmap_core import Application


def main():
    os.environ["PYOPENGL_PLATFORM"] = "glx"

    app = Application()
    app.mainloop()
    app.dispose()


if __name__ == "__main__":
    main()
