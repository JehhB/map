import os


def main():
    os.environ["PYOPENGL_PLATFORM"] = "glx"

    from ratmap_core import Application

    app = Application()
    app.mainloop()
    app.dispose()


if __name__ == "__main__":
    main()
