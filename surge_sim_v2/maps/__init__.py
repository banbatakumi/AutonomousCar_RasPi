"""maps ディレクトリ内のコース定義を自動スキャンする。

各コースモジュール（__init__.py 以外の *.py）は以下を公開する：
    COURSE_NAME : str
    WALLS       : list[tuple[tuple[float,float], tuple[float,float]]]  線分リスト [m]
    START_POSE  : tuple[float, float, float]  (x[m], y[m], heading[deg])
任意で CENTER_LINE : list[tuple[float,float]] を持ってもよい（Phase2用カンニング中心線）。
"""
from __future__ import annotations

import importlib
import pkgutil
from pathlib import Path


def get_all_courses() -> dict[str, dict]:
    """{module_name: {"name", "walls", "start_pose", "center_line"}} を返す。"""
    courses: dict[str, dict] = {}
    pkg_dir = Path(__file__).parent
    for info in pkgutil.iter_modules([str(pkg_dir)]):
        if info.name == "__init__":
            continue
        module = importlib.import_module(f"{__name__}.{info.name}")
        if not hasattr(module, "WALLS") or not hasattr(module, "START_POSE"):
            continue
        courses[info.name] = {
            "name": getattr(module, "COURSE_NAME", info.name),
            "walls": list(module.WALLS),
            "start_pose": tuple(module.START_POSE),
            "center_line": list(getattr(module, "CENTER_LINE", [])),
        }
    return courses
