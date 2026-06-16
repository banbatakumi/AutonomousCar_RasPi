"""コース自動スキャンモジュール。

maps/ ディレクトリ内の *.py（__init__.py を除く）を走査し、
各モジュールから COURSE_NAME / WALLS / START_POSE を読み込む。
"""

from __future__ import annotations

import importlib
import pkgutil
from pathlib import Path

_REQUIRED_ATTRS = ("COURSE_NAME", "WALLS", "START_POSE")


def get_all_courses() -> dict[str, dict]:
    """利用可能な全コースを辞書で返す。

    Returns:
        dict[str, dict]: {COURSE_NAME: {"name", "walls", "start_pose", "module"}}
    """
    courses: dict[str, dict] = {}
    package_dir = Path(__file__).parent

    for mod_info in pkgutil.iter_modules([str(package_dir)]):
        name = mod_info.name
        if name == "__init__":
            continue
        module = importlib.import_module(f"{__name__}.{name}")
        if not all(hasattr(module, attr) for attr in _REQUIRED_ATTRS):
            continue
        courses[module.COURSE_NAME] = {
            "name": module.COURSE_NAME,
            "walls": module.WALLS,
            "start_pose": module.START_POSE,
            # 中心線は任意（Phase2の経路追従用カンニング経路）
            "center_line": getattr(module, "CENTER_LINE", None),
            "module": name,
        }

    return courses
