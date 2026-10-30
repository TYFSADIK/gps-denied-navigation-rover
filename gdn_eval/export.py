"""Export a ROS 2 bag to est.csv / truth.csv without a ROS install (uses the `rosbags` package).

    pip install rosbags
    python -m gdn_eval.export run05_bag/ est.csv truth.csv
"""
import csv
import sys

from .trajectory import latlon_to_enu


def main(bag, est_out, truth_out, est_topic="/odometry/filtered", fix_topic="/ublox/fix"):
    from rosbags.highlevel import AnyReader
    from pathlib import Path
    est, truth, ref = [], [], None
    with AnyReader([Path(bag)]) as r:
        conns = [c for c in r.connections if c.topic in (est_topic, fix_topic)]
        for conn, ts, raw in r.messages(connections=conns):
            m = r.deserialize(raw, conn.msgtype)
            t = ts * 1e-9
            if conn.topic == est_topic:
                est.append((t, m.pose.pose.position.x, m.pose.pose.position.y))
            elif m.status.status >= 0:
                ref = ref or (m.latitude, m.longitude)
                truth.append((t, *latlon_to_enu(m.latitude, m.longitude, *ref)))
    for path, rows in ((est_out, est), (truth_out, truth)):
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t", "x", "y"])
            w.writerows(rows)


if __name__ == "__main__":
    main(*sys.argv[1:4])
