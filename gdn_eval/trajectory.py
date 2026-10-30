"""Drift evaluation against u-blox GNSS truth (ATE / RPE with Umeyama alignment).

Typical use after a run:
    ros2 bag record /odometry/filtered /ublox/fix ...
    python -m gdn_eval.export bag_dir est.csv truth.csv   (see export.py)
    python -m gdn_eval.trajectory est.csv truth.csv --cut 120
"""
import argparse
import csv
import math

import numpy as np


def load_csv(path):
    """columns: t, x, y  (metres, local ENU)."""
    with open(path) as fh:
        r = list(csv.DictReader(fh))
    return np.array([[float(d["t"]), float(d["x"]), float(d["y"])] for d in r])


def latlon_to_enu(lat, lon, lat0, lon0):
    R = 6378137.0
    return (math.radians(lon - lon0) * R * math.cos(math.radians(lat0)), math.radians(lat - lat0) * R)


def associate(est, truth, max_dt=0.1):
    """Nearest-timestamp matching. Returns aligned (N,2) arrays and the times."""
    idx = np.searchsorted(truth[:, 0], est[:, 0]).clip(1, len(truth) - 1)
    left, right = truth[idx - 1, 0], truth[idx, 0]
    idx -= (est[:, 0] - left) < (right - est[:, 0])
    ok = np.abs(truth[idx, 0] - est[:, 0]) <= max_dt
    return est[ok, 0], est[ok, 1:3], truth[idx[ok], 1:3]


def umeyama_2d(src, dst):
    """Best rigid transform (R, t) mapping src -> dst (no scale)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    C = (dst - mu_d).T @ (src - mu_s) / len(src)
    U, _, Vt = np.linalg.svd(C)
    S = np.eye(2)
    if np.linalg.det(U @ Vt) < 0:
        S[1, 1] = -1
    R = U @ S @ Vt
    return R, mu_d - R @ mu_s


def ate(est, truth, align_until=None):
    """Absolute trajectory error. Align on the GNSS-available segment only (honest cut test)."""
    n = len(est) if align_until is None else max(align_until, 3)
    R, t = umeyama_2d(est[:n], truth[:n])
    aligned = est @ R.T + t
    err = np.linalg.norm(aligned - truth, axis=1)
    return err, aligned


def rpe(est, truth, step):
    """Relative pose error over `step` samples (translation only)."""
    d_est = est[step:] - est[:-step]
    d_tru = truth[step:] - truth[:-step]
    return np.linalg.norm(d_est - d_tru, axis=1)


def drift_report(t, est, truth, cut_t):
    """Error growth after the GNSS cut: error at +30/60/120 s and drift as % of distance travelled."""
    cut_i = int(np.searchsorted(t, cut_t))
    err, _ = ate(est, truth, align_until=cut_i)
    dist = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(truth, axis=0), axis=1))])
    rep = {"rmse_before_cut_m": float(np.sqrt(np.mean(err[:cut_i] ** 2))) if cut_i else None}
    for dt in (30, 60, 120):
        j = int(np.searchsorted(t, cut_t + dt))
        if j < len(t):
            travelled = dist[j] - dist[cut_i]
            rep[f"err_at_+{dt}s_m"] = round(float(err[j]), 3)
            rep[f"drift_pct_+{dt}s"] = round(100 * float(err[j]) / travelled, 2) if travelled > 0 else None
    rep["final_err_m"] = round(float(err[-1]), 3)
    return rep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("est")
    ap.add_argument("truth")
    ap.add_argument("--cut", type=float, required=True, help="time (s) GNSS was cut")
    a = ap.parse_args()
    e, tr = load_csv(a.est), load_csv(a.truth)
    t, es, ts = associate(e, tr)
    for k, v in drift_report(t - t[0], es, ts, a.cut).items():
        print(f"{k}: {v}")


if __name__ == "__main__":
    main()
