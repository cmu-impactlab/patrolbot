"""Pure physical trim regressions; no ROS/device access."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from joystick_policy import trim_speed


class RuntimeTrimTests(unittest.TestCase):
    def test_trim_never_exceeds_selected_non_step_ceiling(self):
        for ceiling, step, minimum in [(.165,.05,.05),(.333,.1,.1),(.05,.05,.05),(.7,.1,.1)]:
            value=ceiling
            for _ in range(20):
                value=trim_speed(value,step,minimum,ceiling)
                self.assertLessEqual(value,ceiling)
                self.assertEqual(value,ceiling)

    def test_down_never_raises_stale_low_trim(self):
        self.assertEqual(trim_speed(.01,-.05,.05,.165),.01)
        self.assertEqual(trim_speed(.04,-.1,.1,.333),.04)

    def test_trim_is_bounded_both_directions(self):
        self.assertEqual(trim_speed(.3,.05,.05,.1),.1)
        self.assertEqual(trim_speed(.2,-.05,.05,.1),.1)
        self.assertAlmostEqual(trim_speed(.1,.05,.05,.3),.15)
        self.assertEqual(trim_speed(.1,-.05,.05,.3),.05)


if __name__ == '__main__': unittest.main()
