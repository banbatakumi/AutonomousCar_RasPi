"""ランチャーの登録表が実在のアプリを指しているか。"""

from __future__ import annotations

import importlib.util

from tools.launcher import APPS, ROOT, Options, course_names


def test_every_app_target_exists():
    for spec in APPS:
        argv = spec.argv(Options(course="normal"))
        if argv[0] == "-m":
            assert importlib.util.find_spec(argv[1]) is not None, spec.key
        else:
            assert (ROOT / argv[0]).is_file(), spec.key


def test_keys_unique():
    keys = [s.key for s in APPS]
    assert len(keys) == len(set(keys))


def test_courses_listed():
    assert "normal" in course_names()


def test_sim_argv():
    sim = next(s for s in APPS if s.key == "sim")
    assert sim.argv(Options(course="normal")) == ["-m", "sim.run", "--course", "normal"]
    assert sim.argv(Options(course="normal", browser=False))[-1] == "--no-browser"


def test_editor_argv():
    ed = next(s for s in APPS if s.key == "editor")
    assert ed.argv(Options()) == ["-m", "sim.editor"]
    assert ed.argv(Options(course="normal")) == ["-m", "sim.editor", "normal"]
