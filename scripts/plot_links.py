import csv

import matplotlib.pyplot as plt
import numpy as np


def load_fk_csv(path):
    xs, ys, zs = [], [], []
    rolls, pitches, yaws = [], [], []

    with open(path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            xs.append(float(row["x"]))
            ys.append(float(row["y"]))
            zs.append(float(row["z"]))

            # these are in degrees in your CSV
            rolls.append(float(row["roll"]))
            pitches.append(float(row["pitch"]))
            yaws.append(float(row["yaw"]))

    return (
        np.array(xs),
        np.array(ys),
        np.array(zs),
        np.array(rolls),
        np.array(pitches),
        np.array(yaws),
    )


def rpy_to_rot(roll, pitch, yaw):
    """
    roll, pitch, yaw in *radians*.
    Match Eigen's eulerAngles(0,1,2), which corresponds to R = Rx * Ry * Rz.
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rx = np.array(
        [
            [1, 0, 0],
            [0, cr, -sr],
            [0, sr, cr],
        ]
    )

    Ry = np.array(
        [
            [cp, 0, sp],
            [0, 1, 0],
            [-sp, 0, cp],
        ]
    )

    Rz = np.array(
        [
            [cy, -sy, 0],
            [sy, cy, 0],
            [0, 0, 1],
        ]
    )

    # Eigen: R = Rx * Ry * Rz for eulerAngles(0,1,2)
    return Rx @ Ry @ Rz


def plot_robot(xs, ys, zs, rolls, pitches, yaws):
    fig = plt.figure()
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

    # --- END EFFECTOR ORIENTATION ARROW ---

    roll = np.deg2rad(rolls[-1])
    pitch = np.deg2rad(pitches[-1])
    yaw = np.deg2rad(yaws[-1])

    R = rpy_to_rot(roll, pitch, yaw)

    # Choose an axis to draw. Common choice = local Z axis
    direction = R[:, 2]  # world direction EE is pointing

    ax.quiver(
        xs[-1],
        ys[-1],
        zs[-1],
        direction[0],
        direction[1],
        direction[2],
        length=0.1,  # adjust scale
        normalize=True,
        color="red",
    )

    plt.title("Robot Forward Kinematics with EE Orientation Arrow")
    plt.show()


if __name__ == "__main__":
    xs, ys, zs, rolls, pitches, yaws = load_fk_csv("../build/examples/forward_kinematics.csv")
    plot_robot(xs, ys, zs, rolls, pitches, yaws)
