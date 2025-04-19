from __future__ import annotations

import os
import traceback
from copy import deepcopy
from typing import Any, Dict

import yaml
from platformdirs import PlatformDirs
from reactivex import operators
from reactivex.abc import DisposableBase
from reactivex.subject import BehaviorSubject
from typing_extensions import TypedDict, Unpack

_GET_KWARGS = TypedDict("_GET_KWARGS", {"default": Any}, total=False)


class Config:
    APP_NAME: str = "ratmap"
    APP_AUTHOR: str = "ratmap"

    __dirs: PlatformDirs
    __config: BehaviorSubject[Dict[str, Any]]
    __save_disposer: DisposableBase

    def __init__(self) -> None:
        self.__dirs = PlatformDirs(Config.APP_NAME, Config.APP_AUTHOR)
        self.__config = BehaviorSubject({})

        self.__load()

        self.__save_disposer = self.__config.pipe(operators.debounce(5)).subscribe(
            self.__save
        )

    @property
    def current(self):
        return self.__config.value

    @property
    def config_path(self):
        config_dir = self.__dirs.user_config_dir
        config_path = os.path.join(config_dir, "config.yaml")
        return config_path

    def __load(self):
        path = self.config_path
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)

            if os.path.exists(path):
                with open(path, "r") as f:
                    self.__config.on_next(yaml.safe_load(f) or {})
            else:
                self.__config.on_next({})
        except:
            traceback.print_exc()
            self.__config.on_next({})

    def __save(self, value: Dict[str, Any]):
        path = self.config_path
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)

            with open(path, "w") as f:
                yaml.safe_dump(value, f)
        except:
            traceback.print_exc()

    def __get_value(
        self, value: Dict[str, Any], key: str, **kwargs: Unpack[_GET_KWARGS]
    ) -> Any:
        keys = key.split(".")
        curr = value
        for k in keys:
            if isinstance(curr, dict) and k in curr:
                curr = curr[k]
            else:
                if "default" in kwargs:
                    return kwargs["default"]
                raise KeyError(f"Key '{key}' not found")
        return curr

    def get(self, key: str, **kwargs: Unpack[_GET_KWARGS]) -> Any:
        return self.__get_value(self.current, key, **kwargs)

    def watch(self, key: str, **kwargs: Unpack[_GET_KWARGS]) -> Any:
        return self.__config.pipe(
            operators.map(lambda value: self.__get_value(value, key, **kwargs))
        )

    def update(self, key_path: str, value: Any):
        keys = key_path.split(".")
        config_copy = deepcopy(self.__config.value)

        # Navigate to the nested location
        d = config_copy
        for k in keys[:-1]:
            if k not in d:
                d[k] = {}
            elif not isinstance(d[k], dict):
                raise TypeError(f"Cannot set key '{key_path}': '{k}' is not a dict")
            d = d[k]

        d[keys[-1]] = value
        self.__config.on_next(config_copy)

    def dispose(self):
        self.__config.dispose()
        self.__save_disposer.dispose()
