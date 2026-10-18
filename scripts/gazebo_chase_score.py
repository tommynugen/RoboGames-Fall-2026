"""Scores a run of gazebo_chase_test.py on test4.

    python3 scripts/gazebo_chase_score.py RUN.csv [--plot RUN.png]

Prints when the hunter first reached robot 2 (its front within 6 cm, on the
same level), how high it got, any falls (2 cm down in 0.2 s), its worst
tilt, the longest it got nowhere while not with robot 2, how far it got
from the middle of each leg of the tower's spiral, and how it went through
any corner slot of the box of walls: how far its heading was off the slot's
diagonal and its centre off the slot's middle.
"""

import argparse
import csv
import math

import numpy as np

LANE = 0.1524
SLOT = 0.343


def score(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        return "no rows"
    col = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    t, hx, hy, hz = col["t"], col["hx"], col["hy"], col["hz"]
    near = (col["gap"] < 0.06) & (np.abs(hz - col["tz"]) < 0.02) & (t > 1.0)
    first = f"{t[np.argmax(near)]:.1f} s" if near.any() else "never"

    falls = []
    for k in range(len(t)):
        j = np.searchsorted(t, t[k] + 0.2)
        if (j < len(t) and hz[k] - hz[j] > 0.02
                and (not falls or t[k] - falls[-1] > 1.0)):
            falls.append(round(float(t[k]), 1))
    tilt = math.degrees(np.max(np.maximum(np.abs(col["hroll"]),
                                          np.abs(col["hpitch"]))))

    worst, since, at = 0.0, t[0], (hx[0], hy[0])
    for k in range(len(t)):
        if near[k] or math.dist((hx[k], hy[k]), at) > 0.03:
            since, at = t[k], (hx[k], hy[k])
        worst = max(worst, t[k] - since)

    notes = []
    legs = (
        ("leg 1", (np.abs(hy + LANE) < 0.05) & (hx > -0.10) & (hx < 0.20)
         & (hz > 0.03), hy + LANE),
        ("leg 2", (np.abs(hx + LANE) < 0.05) & (np.abs(hy) < 0.10)
         & (hz > 0.08), hx + LANE),
        ("leg 3", (np.abs(hy - LANE) < 0.05) & (np.abs(hx) < 0.10)
         & (hz > 0.13), hy - LANE),
        ("leg 4", (np.abs(hx - LANE) < 0.05) & (hy > 0.0) & (hy < 0.10)
         & (hz > 0.18), hx - LANE))
    for name, on, off in legs:
        if on.sum() > 3:
            notes.append(f"{name} {100 * np.max(np.abs(off[on])):.1f} cm "
                         f"off its middle")

    for sx in (-1, 1):
        for sy in (-1, 1):
            through = np.hypot(hx - sx * SLOT, hy - sy * SLOT) < 0.025
            if not through.any():
                continue
            axis = math.atan2(sy, sx)
            off = np.degrees(np.abs(np.arcsin(np.sin(col["hyaw"] - axis))))
            across = ((hx - sx * SLOT) * -math.sin(axis)
                      + (hy - sy * SLOT) * math.cos(axis))
            notes.append(
                f"through the {'SN'[sy > 0]}{'WE'[sx > 0]} slot at "
                f"{t[np.argmax(through)]:.1f} s, {off[through].max():.0f} "
                f"degrees off its line, "
                f"{100 * np.abs(across[through]).max():.1f} cm off its "
                f"middle")

    return (f"{t[-1]:.1f} s, reached {first}, up to {hz.max():.3f} m "
            f"(robot 2 at {col['tz'][-1]:.3f}), falls {len(falls)} {falls}, "
            f"tilt {tilt:.0f} degrees, longest stall {worst:.1f} s"
            + "".join(f"; {n}" for n in notes))


def plot(path, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = list(csv.DictReader(open(path)))
    col = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(col["tx"], col["ty"], "-", color="tab:blue", lw=1)
    points = ax.scatter(col["hx"], col["hy"], c=col["t"], cmap="autumn", s=3)
    ax.set_xlim(-0.61, 0.61)
    ax.set_ylim(-0.61, 0.61)
    ax.set_aspect("equal")
    plt.colorbar(points, ax=ax, fraction=0.03, label="s")
    ax.set_title(path.split("/")[-1])
    fig.savefig(out, dpi=70, bbox_inches="tight")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("run")
    parser.add_argument("--plot")
    args = parser.parse_args()
    print(f"{args.run.split('/')[-1]}: {score(args.run)}")
    if args.plot:
        plot(args.run, args.plot)


if __name__ == "__main__":
    main()
