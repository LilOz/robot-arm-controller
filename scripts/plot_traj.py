import csv
import numpy as np
import matplotlib.pyplot as plt


def load_traj_csv(path: str):
    t_ms, x, y, z = [], [], [], []
    qw, qx, qy, qz = [], [], [], []

    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        required = {"t_ms", "x", "y", "z", "qw", "qx", "qy", "qz"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"CSV missing columns: {sorted(missing)}")

        for row in reader:
            t_ms.append(float(row["t_ms"]))
            x.append(float(row["x"]))
            y.append(float(row["y"]))
            z.append(float(row["z"]))
            qw.append(float(row["qw"]))
            qx.append(float(row["qx"]))
            qy.append(float(row["qy"]))
            qz.append(float(row["qz"]))

    return (
        np.array(t_ms) / 1000.0,  # seconds
        np.array(x),
        np.array(y),
        np.array(z),
        np.array(qw),
        np.array(qx),
        np.array(qy),
        np.array(qz),
    )


def plot_traj_3d(x, y, z):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(x, y, z, linewidth=2)
    ax.scatter([x[0]], [y[0]], [z[0]], s=30, label="start")
    ax.scatter([x[-1]], [y[-1]], [z[-1]], s=30, label="end")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("End-effector path (3D)")
    ax.legend()
    plt.show()


def plot_xyz_vs_time(t, x, y, z):
    plt.figure()
    plt.plot(t, x, label="x")
    plt.plot(t, y, label="y")
    plt.plot(t, z, label="z")
    plt.xlabel("time (s)")
    plt.ylabel("position")
    plt.title("Position vs time")
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    t, x, y, z, qw, qx, qy, qz = load_traj_csv("../build/trajectory.csv")
    plot_traj_3d(x, y, z)
    plot_xyz_vs_time(t, x, y, z)

