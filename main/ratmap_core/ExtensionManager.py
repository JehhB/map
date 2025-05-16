import os
import shutil
import sys
import traceback
import zipfile
from fileinput import filename
from glob import glob
from importlib import import_module
from os.path import basename
from pathlib import Path
from tempfile import TemporaryDirectory
from tkinter import TclError
from typing import TYPE_CHECKING, Dict

import yaml
from platformdirs import PlatformDirs
from typing_extensions import override

from ratmap_common import EventTarget

CURRENT_DIR = os.path.realpath(os.path.dirname(__file__))
BUILTIN_EXTENSIONS_FOLDER = os.path.join(CURRENT_DIR, "..", "builtin_extensions")

if TYPE_CHECKING:
    from .Application import Application
    from .BaseExtension import BaseExtension


class ExtensionManager(EventTarget):
    __extensions: "Dict[str, BaseExtension]"
    __context: "Application"

    def __init__(self, context: "Application") -> None:
        super().__init__()
        self.__context = context
        self.__extensions = dict()
        self.add_path(BUILTIN_EXTENSIONS_FOLDER)
        self.__extra_extensions = (
            PlatformDirs("ratmap", "ratmap").user_data_path / "extensions"
        )
        self.__extra_extensions.mkdir(parents=True, exist_ok=True)
        self.add_path(str(self.__extra_extensions))

    def add_path(self, path: str):
        yaml_glob = os.path.join(path, "*.yaml")
        yaml_files = glob(yaml_glob)

        for file in yaml_files:
            self.__load_description(file)

    def add_zip(self, path: str):
        with TemporaryDirectory() as td:
            with zipfile.ZipFile(path, "r") as zip:
                zip.extractall(td)

                yaml_glob = os.path.join(td, "*.yaml")
                yaml_files = glob(yaml_glob)
                filenames = [basename(f) for f in yaml_files]

                print(yaml_glob)

                temp = Path(td)
                for item in temp.iterdir():
                    dst = self.__extra_extensions / item.name
                    src = str(item)

                    print(item)

                    if os.path.exists(dst):
                        if os.path.isdir(dst):
                            shutil.rmtree(dst)
                        else:
                            os.remove(dst)
                    shutil.move(src, dst)

                for f in filenames:
                    path = str(self.__extra_extensions / f)
                    self.__load_description(path)

    def __load_description(self, path: str):
        try:
            with open(path, "r") as file:
                extensions = yaml.safe_load(file)

            base_path = os.path.abspath(os.path.dirname(path))
            for key, description in extensions.items():
                self.__load_extension(
                    key, base_path, description["module"], description["instance"]
                )

        except Exception as e:
            raise ValueError(f"Failed to load extension description: {e}")

    def __load_extension(
        self, key: str, base_path: str, module: str, instance: str
    ) -> None:
        try:

            if base_path not in sys.path:
                sys.path.append(base_path)

            module_ = import_module(module)
            extension: "BaseExtension" = getattr(module_, instance)
            extension.context = self.__context
            extension.extension_manager = self

            extension.parent = self

            self.__extensions[key] = extension

            try:
                if self.__context.config.get(
                    f"{extension.config_namespace}.enabled", default=False
                ):
                    extension.start()
            except:
                traceback.print_exc()

        except Exception as e:
            raise RuntimeError(
                f"Failed to load extension {key} from {module}.{instance}: {e}"
            )

    def items(self):
        return self.__extensions.items()

    def get(self, key: str):
        if key in self.__extensions:
            return self.__extensions[key]
        else:
            raise RuntimeError(f'Extension with id "{key}" is missing')

    @override
    def dispose(self) -> None:
        for _k, x in self.__extensions.items():
            try:
                if x.is_started:
                    _ = x.stop()
            except TclError:
                pass
            except:
                traceback.print_exc()

        return super().dispose()
