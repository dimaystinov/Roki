import unittest
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from sim_scene import recolor_field, validate_settings


class SceneSettingsTests(unittest.TestCase):
    def test_solid_and_stripes_keep_white_paint(self):
        source = np.full((12, 18, 3), (20, 120, 30), dtype=np.uint8)
        source[:, 8:10] = (245, 245, 245)
        solid = recolor_field(source, "solid", (20, 100, 40), (50, 140, 50), 3)
        striped = recolor_field(source, "stripes", (20, 100, 40), (50, 140, 50), 3)
        self.assertTrue(np.all(solid[:, 8:10] == source[:, 8:10]))
        self.assertEqual(solid[5, 1].tolist(), solid[5, 5].tolist())
        self.assertNotEqual(striped[5, 1].tolist(), striped[5, 7].tolist())

    def test_rejects_invalid_brightness_and_style(self):
        with self.assertRaises(ValueError):
            validate_settings({"field": {"style": "dots"}})
        with self.assertRaises(ValueError):
            validate_settings({"lighting": {"sun": -1}})

    def test_person_and_layout_can_be_configured(self):
        settings = validate_settings({
            "person": {"enabled": True, "x_m": 1.5, "walk_amplitude_m": .5},
            "layout": {"field_length_m": 3.35, "field_width_m": 2.35},
        })
        self.assertEqual(settings["person"]["x_m"], 1.5)
        self.assertEqual(settings["layout"]["field_length_m"], 3.35)


if __name__ == "__main__":
    unittest.main()


def test_painted_circle_matches_strategy_and_goal_lines():
    import mujoco
    import sim_roki2 as sim
    from sim_referee import painted_limits
    settings = sim.load_settings(sim.SETTINGS)
    m = mujoco.MjModel.from_xml_path(str(sim.SCENE))
    sim.apply_settings(m, settings)
    turf = m.geom('floor').size[:2]*2
    np.testing.assert_allclose(turf, (4, 3))
    w, h = m.texture('fieldtex').width.item(), m.texture('fieldtex').height.item()
    rgb = m.texture('fieldtex').data.reshape(h,w,3)
    x = (np.arange(w)-w/2)*turf[0]/w
    paint = rgb[h//2].min(axis=1)>205
    ring = x[paint & (x>.2) & (x<.4)]
    expected = sim.sim_parameters(settings)['field.geometry']
    np.testing.assert_allclose([2*ring.mean(),np.ptp(ring)],
                               [expected['circle_diameter'],.05],atol=.004)
    np.testing.assert_allclose(painted_limits(settings['layout']), [1.70,1.20])
    for name, sign in [('yellow',1),('blue',-1)]:
        np.testing.assert_allclose(m.body('goal_'+name).pos[0], sign*1.70)
