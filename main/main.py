import os


def main():
    os.environ["PYOPENGL_PLATFORM"] = "glx"
    os.environ["SDL_JOYSTICK_DEVICE"] = "/dev/input/js0"
    os.environ["SDL_VIDEODRIVER"] = "dummy"

    from ratmap_core import Application

    app = Application()
    app.mainloop()


if __name__ == "__main__":
    main()
