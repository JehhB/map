import os


def main():
    os.environ["PYOPENGL_PLATFORM"] = "glx"

    import app

    app.application.mainloop()


if __name__ == "__main__":
    main()
