import csv

import matplotlib.pyplot as plt
import numpy as np


def load_fk_csv(path):
    xs, ys, zs = [], [], []
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            xs.append(float(row["x"]))
            ys.append(float(row["y"]))
            zs.append(float(row["z"]))
    return np.array(xs), np.array(ys), np.array(zs)


def plot_robot(xs, ys, zs):
    plt.figure()
    
    ax = plt.axes(projection="3d")

    # Draw each link as its own line segment
    for i in range(len(xs) - 1):
        ax.plot(xs[i : i + 2], ys[i : i + 2], zs[i : i + 2], linewidth=3)

    # Joints
    ax.scatter(xs, ys, zs)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    # Equal aspect ratio
    max_range = (
        np.array([xs.max() - xs.min(), ys.max() - ys.min(), zs.max() - zs.min()]).max()
        / 2.0
    )

    mid_x = (xs.max() + xs.min()) * 0.5
    mid_y = (ys.max() + ys.min()) * 0.5
    mid_z = (zs.max() + zs.min()) * 0.5

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.title("Robot Forward Kinematics (Per-Link Colours)")
    plt.show()


if __name__ == "__main__":
    xs, ys, zs = load_fk_csv("../build/forward_kinematics.csv")
    plot_robot(xs, ys, zs)
