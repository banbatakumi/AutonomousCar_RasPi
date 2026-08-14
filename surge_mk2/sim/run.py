"""シミュレータを1コマンドで立ち上げる。

    .venv/bin/python -m sim.run                      # これだけ
    .venv/bin/python -m sim.run --course slalom      # コース指定（名前だけでよい）
    .venv/bin/python -m sim.run --no-gui             # 俯瞰ビュー無し
    .venv/bin/python -m sim.run --list               # コース一覧

`io_node --sim` / `telemetry_node --no-camera` / `planning_node` / `sim.gui` の 4 つを
子プロセスとして起動し、出力に印を付けて1つの画面に流す。
**Ctrl-C か、俯瞰ビューの窓を閉じると全部止まる。**

俯瞰ビューが**異常終了したときだけ**は巻き添えにしない（二重起動や描画のバグで
落ちただけなら、走っている車まで止める理由が無い）。終了コードで区別している。

## 立ち上げ前に、詰まりどころを潰してから起動する

`PROGRESS.md`「Mac で起動できないときに疑う2つ」に書いた罠を、ここで自動的に処理する。
**エラーを分かりやすくするのではなく、そもそも踏ませない**のが狙い。

1. **Rosetta（x86_64）で起動された** → arm64 で自分を起動し直す。
   放っておくと `msgspec` の `incompatible architecture` という原因の分かりにくい
   ImportError になる
2. **ポート 8000 を古い `telemetry_node` が掴んでいる** → 起動時刻つきで報告して止まる。
   実際に4日前のプロセスが居座り、古い GUI を配っていたことがある
3. **`io_node` / `bus_demo` / `replay_node` が既に動いている** → 同じ `io` endpoint を
   取り合うので、見つけたら止める

このファイルは **stdlib だけで書く**。numpy や pygame を import すると、
1 番の診断をする前に ImportError で落ちてしまう。
"""

from __future__ import annotations

import argparse
import os
import platform
import signal
import socket
import subprocess
import sys
import threading
import time
import webbrowser
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
COURSE_DIR = ROOT / "sim" / "courses"
PORT = 8000

#: 再実行のループ防止
ENV_REEXEC = "SURGE_ARCH_REEXEC"

C = {"io": "\033[36m", "tele": "\033[35m", "gui": "\033[32m",
     "run": "\033[33m", "err": "\033[31m", "dim": "\033[2m", "off": "\033[0m"}


def say(tag: str, msg: str) -> None:
    print(f"{C.get(tag, '')}[{tag:4s}]{C['off']} {msg}", flush=True)


# ── 起動前の点検 ────────────────────────────────────────────────────────

def ensure_arm64() -> None:
    """Rosetta で起動されていたら arm64 で自分を起動し直す。

    python.org の Python は universal2 で **親プロセスのアーキを継承する**ため、
    Rosetta のターミナルから起動すると x86_64 で動き、venv の arm64 ホイールを
    読めない。ここで直しておかないと `msgspec` の import で落ちる。
    """
    if platform.machine() != "x86_64" or os.environ.get(ENV_REEXEC):
        return
    try:
        apple_silicon = subprocess.run(
            ["sysctl", "-n", "hw.optional.arm64"],
            capture_output=True, text=True).stdout.strip() == "1"
    except Exception:
        apple_silicon = False
    if not apple_silicon:
        return                       # 本物の Intel Mac。そのまま動かす

    say("run", f"{C['dim']}Rosetta(x86_64) で起動されたので arm64 で立て直します"
               f"（恒久対応は Terminal.app の「Rosetta を使用して開く」を外す）{C['off']}")
    os.environ[ENV_REEXEC] = "1"
    os.execv("/usr/bin/arch",
             ["arch", "-arm64", sys.executable, "-m", "sim.run", *sys.argv[1:]])


def port_holder(port: int) -> tuple[int, str] | None:
    """`port` を LISTEN しているプロセスの (PID, 起動時刻)。空いていれば None。"""
    with socket.socket() as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("127.0.0.1", port))
            return None
        except OSError:
            pass
    try:
        pid = subprocess.run(["lsof", "-t", f"-iTCP:{port}", "-sTCP:LISTEN"],
                             capture_output=True, text=True).stdout.split()[0]
        started = subprocess.run(["ps", "-o", "lstart=", "-p", pid],
                                 capture_output=True, text=True).stdout.strip()
        return int(pid), started
    except Exception:
        return -1, "不明"


def bus_rivals() -> list[tuple[int, str, str]]:
    """endpoint を取り合う相手（io_node / bus_demo / replay_node / planning_node）。

    **起動時刻を必ず出す。** 実際に4日前の `bus_demo --faults` が動きっぱなしで、
    その偽データが GUI に出ていたことがある。コマンド名だけ見ても気づけない。
    """
    # `SURGE_BUS_DIR` で別のバスを指しているなら endpoint を取り合わない。
    # 2つのスタックを意図的に並走させる場合（テスト等）に誤検出しないため
    if os.environ.get("SURGE_BUS_DIR"):
        return []
    out = subprocess.run(["ps", "-eo", "pid,lstart,command"],
                         capture_output=True, text=True).stdout.splitlines()
    me = os.getpid()
    rivals = []
    for line in out:
        if not any(k in line for k in ("raspi.nodes.io_node", "raspi.tools.bus_demo",
                                       "raspi.nodes.replay_node",
                                       # planning_node は `planning` endpoint を
                                       # bind するので、残っていると新しい方が上がれない
                                       "raspi.nodes.planning_node")):
            continue
        if "sim.run" in line or "ps -eo" in line:
            continue
        parts = line.split(None, 6)          # pid + lstart(5語) + command
        try:
            pid = int(parts[0])
        except (ValueError, IndexError):
            continue
        if pid == me:
            continue
        started = " ".join(parts[1:6])
        # 長い python のパスを畳んで、何を動かしているかだけ見せる
        cmd = parts[6] if len(parts) > 6 else ""
        cmd = cmd[cmd.find(" -m ") + 1:] if " -m " in cmd else cmd
        rivals.append((pid, started, cmd[:80]))
    return rivals


def udp_holder(port: int) -> int | None:
    """UDP `port` を掴んでいる PID。空いていれば None。"""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("127.0.0.1", port))
            return None
        except OSError:
            pass
    try:
        return int(subprocess.run(["lsof", "-t", f"-iUDP:{port}"],
                                  capture_output=True, text=True).stdout.split()[0])
    except Exception:
        return -1


def preflight(kill_stale: bool, port: int, want_gui: bool) -> bool:
    ok = True

    # シム ↔ シミュレータGUI の私設チャンネル（`sim/channel.py`）。
    # **二重起動でここが埋まると GUI だけ黙って落ちる**ので先に見る
    if want_gui:
        # `sim/channel.py` と同じ既定値。**stdlib だけで書く方針なので import せず読む**
        truth = int(os.environ.get("SURGE_SIM_TRUTH_PORT", 5599))
        ctrl = int(os.environ.get("SURGE_SIM_CTRL_PORT", 5598))
        for p, who in ((truth, "シミュレータGUI"), (ctrl, "シミュレータ本体")):
            pid = udp_holder(p)
            if pid is None:
                continue
            say("err", f"UDP {p} を PID {pid} が使っています（{who}の二重起動）")
            if kill_stale and pid > 0:
                os.kill(pid, signal.SIGTERM)
                say("run", f"PID {pid} を落としました")
                time.sleep(0.6)
            else:
                print(
                    f"        {C['dim']}kill {pid}   / --no-gui / --kill-stale{C['off']}")
                ok = False

    for pid, started, cmd in bus_rivals():
        say("err", f"io endpoint を取り合う相手が動いています（PID {pid}・起動 {started}）")
        print(f"        {C['dim']}{cmd}{C['off']}")
        if kill_stale:
            os.kill(pid, signal.SIGTERM)
            say("run", f"PID {pid} を落としました")
            time.sleep(0.6)
        else:
            print(
                f"        {C['dim']}kill {pid}   もしくは --kill-stale を付けて再実行{C['off']}")
            ok = False

    held = port_holder(port)
    if held:
        pid, started = held
        say("err", f"ポート {port} を PID {pid} が使っています（起動: {started}）")
        if kill_stale and pid > 0:
            os.kill(pid, signal.SIGTERM)
            say("run", f"PID {pid} を落としました")
            time.sleep(0.6)
            if port_holder(port):
                say("err", "まだ空きません。kill -9 が要るかもしれません")
                ok = False
        else:
            print(f"        {C['dim']}古い telemetry_node が残っていることが多い。"
                  f"起動時刻が古ければまず疑う{C['off']}")
            print(
                f"        {C['dim']}kill {pid}   もしくは --kill-stale を付けて再実行{C['off']}")
            ok = False

    if not (ROOT / "gui" / "dist" / "index.html").exists():
        say("err", "gui/dist が無いのでブラウザに GUI が出ません")
        print(
            f"        {C['dim']}cd gui && npm install && npm run build{C['off']}")
    return ok


# ── コース ──────────────────────────────────────────────────────────────

def _course_files() -> list[Path]:
    """PNG と、PNG を伴わない JSON（センターライン方式）。"""
    pngs = sorted(COURSE_DIR.glob("*.png"))
    stems = {p.stem for p in pngs}
    return pngs + sorted(j for j in COURSE_DIR.glob("*.json") if j.stem not in stems)


def resolve_course(name: str) -> Path:
    """`circuit` でも `sim/courses/circuit.json` でも受ける。"""
    p = Path(name)
    if p.suffix in (".png", ".json") and p.exists():
        return p
    for cand in _course_files():
        if cand.stem == p.stem:
            return cand.relative_to(ROOT) if cand.is_relative_to(ROOT) else cand
    raise SystemExit(f"コースが見つかりません: {name}\n"
                     f"  候補: {', '.join(f.stem for f in _course_files())}")


# ── 子プロセス ──────────────────────────────────────────────────────────

class Child:
    """:param essential: False なら**落ちても他を巻き添えにしない。**
    :param window: この子の正常終了（終了コード 0）を「人が終わりにした」と読む。

    俯瞰ビュー（`sim.gui`）は無くてもシミュレーションは成立するので、
    二重起動や描画のバグで GUI だけ落ちたときに走っている車まで止める理由が無い。
    **ただし正常終了は別扱い**で、それは「窓を閉じた」＝人が終わりにしたという
    意思表示なので全部畳む（`window=True`）。この2つを区別しないと、
    「閉じたのに裏でシムが走り続ける」か「バグで落ちたら全部道連れ」のどちらかになる。

    `planning_node` も `essential=False` だが `window=False`。自動運転の実装を
    いじっている最中に落ちても、**手動で走らせている車まで道連れにしない**
    （自律指令が途絶えれば telemetry_node が制動に落とすので、危険側には転ばない）。
    """

    def __init__(self, tag: str, args: list[str], essential: bool = True,
                 window: bool = False) -> None:
        self.tag = tag
        self.essential = essential
        self.window = window
        self.proc = subprocess.Popen(
            [sys.executable, "-u", "-m", *args], cwd=ROOT,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            start_new_session=True,
            env={**os.environ, "PYGAME_HIDE_SUPPORT_PROMPT": "1"})
        self.thread = threading.Thread(target=self._pump, daemon=True)
        self.thread.start()

    def _pump(self) -> None:
        for raw in iter(self.proc.stdout.readline, b""):
            line = raw.decode("utf-8", "replace").rstrip("\r\n")
            # 子の飾り（"# " や空行）は落として読みやすくする
            if line.strip() in ("", "#"):
                continue
            say(self.tag, line.lstrip("# "))

    def stop(self) -> None:
        if self.proc.poll() is not None:
            return
        # SIGINT を送る。io_node は終了時に DISARM を1発送って畳む作りになっている
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            return
        try:
            self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass


def wait_port(port: int, timeout: float = 8.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        with socket.socket() as s:
            s.settimeout(0.3)
            if s.connect_ex(("127.0.0.1", port)) == 0:
                return True
        time.sleep(0.2)
    return False


# ── 本体 ────────────────────────────────────────────────────────────────

def main() -> int:
    ensure_arm64()

    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--course", default="circuit",
                    help="コース名 or PNG パス（既定 circuit）")
    ap.add_argument("--list", action="store_true", help="コース一覧を出して終わる")
    ap.add_argument("--no-gui", action="store_true", help="pygame の俯瞰ビューを開かない")
    ap.add_argument("--no-planning", action="store_true",
                    help="planning_node を上げない（自動運転タブが使えなくなる）")
    ap.add_argument("--no-browser", action="store_true", help="ブラウザを自動で開かない")
    ap.add_argument("--kill-stale", action="store_true",
                    help="ポートや endpoint を掴んでいる古いプロセスを落としてから起動する")
    ap.add_argument("--safe", action="store_true",
                    help="--allow-arm を付けない（COMMAND が DISARM 固定になる）")
    ap.add_argument("--port", type=int, default=PORT,
                    help=f"WS/HTTP のポート（既定 {PORT}）")
    ap.add_argument("--max-speed", type=float, default=3.0)
    ap.add_argument("--max-steer", type=float, default=1.047)
    args = ap.parse_args()

    if args.list:
        for f in _course_files():
            kind = "センターライン" if f.suffix == ".json" else "PNG"
            print(f"  {f.stem:10s} {kind:12s} {f.relative_to(ROOT)}")
        return 0

    course = resolve_course(args.course)
    if not preflight(args.kill_stale, args.port, not args.no_gui):
        say("err", "起動を中止しました")
        return 2

    # **`--allow-arm` を既定で付ける。** このランチャは `--sim` 専用の入口で、
    # 実車に届く経路が存在しないため。実機を触る io_node は今まで通り明示が要る
    io = ["raspi.nodes.io_node", "--sim", "--course", str(course), "--quiet",
          "--no-heartbeat", "--no-leds",
          "--max-speed", str(args.max_speed), "--max-steer", str(args.max_steer)]
    if not args.safe:
        io.append("--allow-arm")

    say("run", f"コース {course.stem}   速度上限 {args.max_speed} m/s   "
               f"舵角上限 {args.max_steer} rad"
               + ("   [--safe: DISARM 固定]" if args.safe else ""))

    children = [Child("io", io),
                Child("tele", ["raspi.nodes.telemetry_node", "--no-camera",
                               "--port", str(args.port)])]
    # 自動運転（§8）。**engage は GUI の自動運転タブから人間が行う**ので、
    # 上げておくだけでは車は動かない（`--mode` も渡さない）
    if not args.no_planning:
        children.append(Child("plan", ["raspi.nodes.planning_node", "--quiet"],
                              essential=False))
    if not args.no_gui:
        children.append(
            Child("gui", ["sim.gui"], essential=False, window=True))

    url = f"http://localhost:{args.port}/"
    if wait_port(args.port):
        say("run", f"{url}   ラジコンタブで Space 長押し + W / A / D")
        if not args.no_browser:
            webbrowser.open(url)
    else:
        say("err", f"{args.port} が開きません。上の tele の行を見てください")

    # **SIGTERM でも畳めるようにする。** `KeyboardInterrupt` を待つだけだと `kill` で
    # 親だけが死んで子が残る。加えて、バックグラウンド（`&`）起動では SIGINT が
    # SIG_IGN として継承され Ctrl-C も効かなくなるが、ここで上書きすれば効く
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())

    say("run", f"{C['dim']}Ctrl-C で全部止めます{C['off']}")
    reported: set[str] = set()
    while not stop.wait(0.3):
        for c in children:
            if c.proc.poll() is None or c.tag in reported:
                continue
            reported.add(c.tag)
            code = c.proc.returncode
            if c.essential:
                say("err", f"{c.tag} が終了しました（code {code}）。全部止めます")
                stop.set()
            elif c.window and code == 0:
                # **窓を閉じた＝終わりの意思表示。** 端末に戻って Ctrl-C を押させない
                say("run", f"{c.tag} のウィンドウが閉じられました。全部止めます")
                stop.set()
            else:
                # 落ちただけ。車は走っているので巻き添えにしない
                again = ("python -m sim.gui" if c.window
                         else "python -m raspi.nodes.planning_node")
                say("err", f"{c.tag} が終了しました（code {code}）。"
                           f"シミュレーションは続行します"
                           f"（上げ直すなら別ターミナルで {again}）")
    print()
    say("run", "停止中…")
    for c in reversed(children):
        c.stop()
    say("run", "終了")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
