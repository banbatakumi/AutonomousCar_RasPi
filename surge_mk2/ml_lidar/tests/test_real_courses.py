"""実地図→Course変換（`saved_maps/magp_*.npz`が無い環境ではskip）。"""
import pytest

from ml_lidar.real_courses import list_real_course_names, load_real_course


@pytest.mark.skipif(not list_real_course_names(), reason="saved_maps/magp_*.npz が無い")
def test_real_courses_have_closed_centerline_and_free_start():
    from ml_lidar.env import VehicleSpec
    spec = VehicleSpec.load()
    for name in list_real_course_names():
        c = load_real_course(name)
        assert c.centerline is not None and len(c.centerline) >= 8
        assert not c.collides(*c.start, c.body_samples(spec.footprint))
