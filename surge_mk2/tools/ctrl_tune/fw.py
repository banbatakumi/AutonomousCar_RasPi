"""ファーム（`MainF446RE_V3`）の `src/control` をホストでコンパイルして呼び出す。

ファームのリポジトリは既定でこのリポジトリの隣（`../AutonomousCar/Program/MainF446RE_V3`）。
違う場所なら環境変数 `SURGE_FW_DIR` で指す。`host/host_sim.c`（車両モデルと閉ループの本体）と
`host/shim/`（HAL に触る部分の差し替え）は常に作業ツリーのものを使い、`src/control` と
`lib/` だけを作業ツリーか指定のコミット（`git_ref`）から取る——コミット済みのロジックと
書きかけのロジックを同じ車両モデルで比べるため。
"""

from __future__ import annotations

import ctypes
import hashlib
import os
import re
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np

__all__ = ["FW_DIR", "ParamSpec", "param_table", "Firmware", "Session", "HostPlant", "HostConfig",
           "IN", "OUT", "ABI"]

REPO_ROOT = Path(__file__).resolve().parents[2]
FW_DIR = Path(os.environ.get("SURGE_FW_DIR",
                             REPO_ROOT.parent.parent / "AutonomousCar" / "Program" / "MainF446RE_V3"))

#: `host/host_sim.c` の HOST_SIM_ABI
ABI = 4

#: 入力の列（`host_sim.c` の enum と同じ並び）
IN = {name: i for i, name in enumerate(
    ("mode", "value", "accel_limit", "steer", "mu_left", "mu_right", "test_moment", "front_scale"))}
#: 出力の列
OUT = {name: i for i, name in enumerate(
    ("t", "speed", "wheel_left", "wheel_right", "yaw_rate", "cmd_left", "cmd_right",
     "force_left", "force_right", "kappa_left", "kappa_right", "tc_limit_left", "tc_limit_right",
     "abs_limit", "yaw_target", "fw_speed", "fw_slip_left", "fw_slip_right", "flags", "distance",
     "torque_left", "torque_right", "accel", "fw_rear_left", "fw_rear_right", "fw_yaw_rate",
     "front_odom"))}

MODE_SPEED, MODE_TORQUE, MODE_BRAKE, MODE_DISARM = 0, 1, 2, 3
#: 車速指令＋ローンチコントロールの要求（`COMMAND.flags2` の LAUNCH）。持たない古いファームでは
#: ただの車速指令
MODE_SPEED_LAUNCH = 4
FLAG_TC, FLAG_ABS, FLAG_TV, FLAG_LIFT, FLAG_LAUNCH = 1, 2, 4, 8, 16


class HostPlant(ctypes.Structure):
    """`host_sim.c` の HostPlant（車両モデルのパラメータ）。並びを変えたら ABI を上げる。"""

    _fields_ = [(n, ctypes.c_double) for n in (
        "mass_kg", "wheel_inertia_kgm2", "wheel_friction_nm", "wheel_damping_nms",
        "rolling_decel_m_s2",
        "mu", "fz_static_n", "load_transfer", "lateral_transfer", "rear_lateral_share",
        "tyre_b", "tyre_c", "slip_speed_floor_m_s",
        "md_delay_s", "md_tau_s", "md_brake_fade_rad_s", "md_speed_tau_s",
        "yaw_inertia_kgm2", "yaw_damping", "steer_gain", "steer_gain_cubic",
        "lateral_accel_max_m_s2",
        "front_noise_rad_s", "rear_noise_rad_s", "gyro_noise_rad_s", "gyro_tau_s",
        "gyro_bias_rad_s", "front_scale_error", "front_ripple")] + [("seed", ctypes.c_uint32), ("reserved", ctypes.c_uint32)]


class HostConfig(ctypes.Structure):
    _fields_ = [("dt_s", ctypes.c_double), ("substeps", ctypes.c_int32),
                ("tc_enabled", ctypes.c_int32), ("abs_enabled", ctypes.c_int32),
                ("wheel_lift_guard_enabled", ctypes.c_int32), ("tv_enabled", ctypes.c_int32),
                ("imu_ready", ctypes.c_int32), ("initial_speed_m_s", ctypes.c_double)]


class _HostParam(ctypes.Structure):
    _fields_ = [("id", ctypes.c_uint16), ("value", ctypes.c_float)]


@dataclass(frozen=True)
class ParamSpec:
    """`control_params.h` の表の1行。"""

    name: str
    id: int
    default: float
    lo: float
    hi: float


_ROW = re.compile(r"X\(\s*(\w+)\s*,\s*(0x[0-9A-Fa-f]+)\s*,\s*([-\d.eE]+)f?\s*,\s*([-\d.eE]+)f?\s*,"
                  r"\s*([-\d.eE]+)f?\s*\)")


def param_table(fw_dir: Path | None = None) -> dict[str, ParamSpec]:
    """`src/control/control_params.h` の表（名前 → 仕様）。ファームが唯一の定義。"""
    text = ((fw_dir or FW_DIR) / "src" / "control" / "control_params.h").read_text(encoding="utf-8")
    body = text[text.index("#define CONTROL_PARAM_TABLE(X)"):text.index("typedef struct")]
    out = {}
    for m in _ROW.finditer(body):
        out[m.group(1)] = ParamSpec(m.group(1), int(m.group(2), 16), float(m.group(3)),
                                    float(m.group(4)), float(m.group(5)))
    if not out:
        raise RuntimeError("control_params.h の表が読めません")
    return out


_SOURCES = ("src/control/drive.c", "src/control/torque_vectoring.c", "src/control/control_params.c")
_INCLUDES = ("lib/pid", "lib/filter", "lib/mymath", "src/control")


def _cache_dir() -> Path:
    d = Path(tempfile.gettempdir()) / "surge_ctrl_tune"
    d.mkdir(exist_ok=True)
    return d


def _export_ref(git_ref: str) -> Path:
    """`git_ref` の `src/control` と `lib/` を一時ディレクトリへ取り出す。"""
    sha = subprocess.run(["git", "-C", str(FW_DIR), "rev-parse", git_ref], check=True,
                         capture_output=True, text=True).stdout.strip()
    dst = _cache_dir() / f"tree_{sha[:16]}"
    if not dst.exists():
        dst.mkdir()
        prefix = subprocess.run(["git", "-C", str(FW_DIR), "rev-parse", "--show-prefix"], check=True,
                                capture_output=True, text=True).stdout.strip()
        tar = subprocess.run(["git", "-C", str(FW_DIR), "archive", sha, "src/control", *_INCLUDES[:3]],
                             check=True, capture_output=True).stdout
        subprocess.run(["tar", "-x", "-C", str(dst)], input=tar, check=True)
        if prefix and (dst / prefix).exists():      # archive がリポジトリ根からのパスで出した場合
            for child in (dst / prefix).iterdir():
                child.rename(dst / child.name)
    return dst


def _build(tree: Path) -> Path:
    host = FW_DIR / "host"
    srcs = [host / "host_sim.c"] + [tree / s for s in _SOURCES if (tree / s).exists()]
    has_params = (tree / "src/control/control_params.c").exists()
    has_launch = "Drive_SetLaunch" in (tree / "src/control/drive.h").read_text(encoding="utf-8")
    files = srcs + sorted((host / "shim").glob("*.h")) + sorted((tree / "src/control").glob("*.h"))
    digest = hashlib.sha256()
    for f in files:
        digest.update(f.read_bytes())
    ext = "dylib" if sys.platform == "darwin" else "so"
    lib = _cache_dir() / f"host_sim_{digest.hexdigest()[:16]}.{ext}"
    if not lib.exists():
        cmd = ["cc", "-O2", "-shared", "-fPIC", f"-I{host / 'shim'}",
               *[f"-I{tree / inc}" for inc in _INCLUDES],
               *(["-DHOST_FW_HAS_CONTROL_PARAMS"] if has_params else []),
               *(["-DHOST_FW_HAS_LAUNCH"] if has_launch else []),
               *map(str, srcs), "-lm", "-o", str(lib)]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            raise RuntimeError(f"ファームのホスト向けコンパイルに失敗:\n{r.stderr}")
    return lib


class Firmware:
    """ホストでコンパイルしたファームの制御。`git_ref` を渡すとそのコミットの `src/control`。"""

    def __init__(self, git_ref: str | None = None) -> None:
        if not (FW_DIR / "host" / "host_sim.c").exists():
            raise FileNotFoundError(
                f"ファームのリポジトリが見つかりません: {FW_DIR}（環境変数 SURGE_FW_DIR で指定）")
        self.git_ref = git_ref
        tree = _export_ref(git_ref) if git_ref else FW_DIR
        self._lib = ctypes.CDLL(str(_build(tree)))
        self._lib.host_sim_run.restype = ctypes.c_int
        self._lib.host_sim_create.restype = ctypes.c_void_p
        self._lib.host_sim_free.argtypes = [ctypes.c_void_p]
        self._lib.host_sim_step.argtypes = [ctypes.c_void_p, ctypes.c_int,
                                            ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)]
        self._lib.host_sim_set_enables.argtypes = [ctypes.c_void_p] + [ctypes.c_int] * 4
        self._lib.host_sim_set_param.argtypes = [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_float]
        if self._lib.host_sim_abi() != ABI or self._lib.host_sim_in_count() != len(IN) \
                or self._lib.host_sim_out_count() != len(OUT) \
                or self._lib.host_sim_plant_size() != ctypes.sizeof(HostPlant):
            raise RuntimeError("host_sim.c と tools/ctrl_tune/fw.py の構造体・列の並びが合いません")
        #: このファームが調整パラメータ（`control_params.h`）を持つか（持たない古いコミットは定数）
        self.has_params = bool(self._lib.host_sim_has_params())
        self.params = param_table(tree) if self.has_params else {}

    def _kv(self, params: dict[str, float] | None):
        if not params or not self.has_params:
            return (_HostParam * 0)()
        unknown = set(params) - set(self.params)
        if unknown:
            raise KeyError(f"ファームに無いパラメータ: {sorted(unknown)}")
        return (_HostParam * len(params))(*[_HostParam(self.params[k].id, float(v))
                                            for k, v in params.items()])

    def session(self, plant: HostPlant, config: HostConfig,
                params: dict[str, float] | None = None) -> "Session":
        """続きから進められるシミュレーション（planner を閉ループで回すとき）。"""
        return Session(self, plant, config, self._kv(params))

    def run(self, plant: HostPlant, config: HostConfig, inputs: np.ndarray,
            params: dict[str, float] | None = None) -> np.ndarray:
        """`inputs`（n×len(IN)）を入れて n×len(OUT) を返す。`params` は名前 → 値（省略は既定値）。"""
        u = np.ascontiguousarray(inputs, dtype=np.float64)
        if u.ndim != 2 or u.shape[1] != len(IN):
            raise ValueError("inputs の形が違います")
        out = np.zeros((u.shape[0], len(OUT)))
        kv = self._kv(params)
        rc = self._lib.host_sim_run(
            ctypes.byref(plant), ctypes.byref(config), ctypes.c_int(u.shape[0]),
            u.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            out.ctypes.data_as(ctypes.POINTER(ctypes.c_double)), kv, ctypes.c_int(len(kv)))
        if rc != 0:
            raise RuntimeError(f"host_sim_run が失敗しました（{rc}）")
        return out


class Session:
    """`Firmware.session()` が返す、続きから進められるシミュレーション1本。"""

    def __init__(self, fw: Firmware, plant: HostPlant, config: HostConfig, kv) -> None:
        self._fw = fw
        self._lib = fw._lib
        self._h = self._lib.host_sim_create(ctypes.byref(plant), ctypes.byref(config), kv,
                                            ctypes.c_int(len(kv)))
        if not self._h:
            raise RuntimeError("host_sim_create が失敗しました")
        self._enables = [config.tc_enabled, config.abs_enabled, config.wheel_lift_guard_enabled,
                         config.tv_enabled]

    def step(self, inputs: np.ndarray) -> np.ndarray:
        u = np.ascontiguousarray(inputs, dtype=np.float64)
        out = np.zeros((u.shape[0], len(OUT)))
        self._lib.host_sim_step(self._h, u.shape[0], u.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                                out.ctypes.data_as(ctypes.POINTER(ctypes.c_double)))
        return out

    def set_enables(self, tc: bool | None = None, abs_: bool | None = None, lift: bool | None = None,
                    tv: bool | None = None) -> None:
        for i, v in enumerate((tc, abs_, lift, tv)):
            if v is not None:
                self._enables[i] = int(v)
        self._lib.host_sim_set_enables(self._h, *self._enables)

    def set_param(self, name: str, value: float) -> None:
        if self._lib.host_sim_set_param(self._h, self._fw.params[name].id, float(value)) != 0:
            raise KeyError(name)

    def close(self) -> None:
        if self._h:
            self._lib.host_sim_free(self._h)
            self._h = None

    def __del__(self) -> None:
        self.close()


@lru_cache(maxsize=4)
def firmware(git_ref: str | None = None) -> Firmware:
    return Firmware(git_ref)
