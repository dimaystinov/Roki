import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "roki-ng"),
                str(ROOT / "roki-ng-operator-gui/scripts")]

from sim_servo_stand import ServoStand, compare_trace


class ServoStandTests(unittest.TestCase):
    def test_hung_robot_stays_above_field_and_head_moves(self):
        stand = ServoStand(clearance_m=.1)
        stand.command_head(600, frames=20)
        stand.advance(.8)
        self.assertAlmostEqual(stand.data.qpos[2], .39, delta=.001)
        self.assertAlmostEqual(stand.model.body_pos[stand.model.body("calibration_stand").id, 0], -.6)
        self.assertGreater(stand.head_counts, 500)

    def test_comparison_pairs_real_samples_with_simulated_angles(self):
        rows = [dict(kind="command", t=1., pan=0, reply={}),
                dict(kind="sample", t=1.2, pan=0, measured=0, sent=0),
                dict(kind="command", t=1.3, pan=600, reply={}),
                dict(kind="sample", t=1.8, pan=600, measured=900, sent=921)]
        result = compare_trace(rows)
        self.assertEqual(len(result["samples"]), 2)
        self.assertIn("simulated", result["samples"][1])
        self.assertGreater(result["samples"][1]["simulated"], 500)


if __name__ == "__main__":
    unittest.main()
