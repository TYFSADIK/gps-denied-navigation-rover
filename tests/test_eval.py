import unittest

import numpy as np

from gdn_eval.trajectory import ate, drift_report, rpe, umeyama_2d


class TestEval(unittest.TestCase):
    def test_umeyama_recovers_rotation(self):
        rng = np.random.default_rng(0)
        src = rng.normal(size=(50, 2))
        th = 0.7
        R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        Rh, t = umeyama_2d(src, src @ R.T + [3, -2])
        np.testing.assert_allclose(Rh, R, atol=1e-9)

    def test_drift_grows_after_cut(self):
        t = np.arange(0, 300, 0.5)
        truth = np.column_stack([0.5 * t, np.zeros_like(t)])
        est = truth.copy()
        after = t > 100
        est[after, 1] += 0.01 * (t[after] - 100)       # lateral drift after GNSS cut
        rep = drift_report(t, est, truth, cut_t=100)
        self.assertLess(rep["err_at_+30s_m"], rep["err_at_+120s_m"])
        self.assertAlmostEqual(rep["err_at_+120s_m"], 1.2, delta=0.15)

    def test_rpe_zero_on_identical(self):
        x = np.random.default_rng(1).normal(size=(20, 2)).cumsum(0)
        self.assertAlmostEqual(rpe(x, x, 5).max(), 0)


if __name__ == "__main__":
    unittest.main()
