import csv

import matplotlib.pyplot as plt
import numpy as np


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
    ax.scatter([x[0]], [y[0]], [z[0]], label="start")
    ax.scatter([x[-1]], [y[-1]], [z[-1]], label="end")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("End-effector path (3D)")
    ax.legend()


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


def plot_quaternion_vs_time(t, qw, qx, qy, qz):
    for i in range(1, len(qw)):
        dot = (
            qw[i] * qw[i - 1]
            + qx[i] * qx[i - 1]
            + qy[i] * qy[i - 1]
            + qz[i] * qz[i - 1]
        )
        if dot < 0:
            print(f"Quaternion sign flip between {i-1} and {i}")
    plt.figure()
    plt.plot(t, qw, label="qw")
    plt.plot(t, qx, label="qx")
    plt.plot(t, qy, label="qy")
    plt.plot(t, qz, label="qz")
    plt.xlabel("time (s)")
    plt.ylabel("quaternion component")
    plt.title("Quaternion components vs time")
    plt.legend()
    plt.grid(True)


def quat_to_rotvec(qw, qx, qy, qz):
    q = np.array([qw, qx, qy, qz], dtype=float)
    q /= np.linalg.norm(q)

    w = q[0]
    v = q[1:]
    norm_v = np.linalg.norm(v)

    if norm_v < 1e-8:
        return np.zeros(3)

    theta = 2.0 * np.arctan2(norm_v, w)
    axis = v / norm_v
    return axis * theta


def quat_rotate_vec(qw, qx, qy, qz, v):
    q = np.array([qw, qx, qy, qz], dtype=float)
    q /= np.linalg.norm(q)

    w, x, y, z = q
    qv = np.array([x, y, z])

    # Rodrigues-style formula
    return v + 2.0 * np.cross(qv, np.cross(qv, v) + w * v)


def plot_rotation_vector_vs_time(t, qw, qx, qy, qz):
    rotvecs = np.array(
        [quat_to_rotvec(qw[i], qx[i], qy[i], qz[i]) for i in range(len(t))]
    )

    plt.figure()
    plt.plot(t, rotvecs[:, 0], label="ωx")
    plt.plot(t, rotvecs[:, 1], label="ωy")
    plt.plot(t, rotvecs[:, 2], label="ωz")
    plt.xlabel("time (s)")
    plt.ylabel("rotation (rad)")
    plt.title("Angle–axis rotation vector vs time")
    plt.legend()
    plt.grid(True)


def plot_traj_3d_with_orientation(
    x,
    y,
    z,
    qw,
    qx,
    qy,
    qz,
    stride=10,
    axis=np.array([0.0, 0.0, 1.0]),
    scale=0.05,
):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(x, y, z, linewidth=2, label="path")
    ax.scatter([x[0]], [y[0]], [z[0]], label="start")
    ax.scatter([x[-1]], [y[-1]], [z[-1]], label="end")

    xs, ys, zs = [], [], []
    us, vs, ws = [], [], []

    for i in range(0, len(x), stride):
        d = quat_rotate_vec(qw[i], qx[i], qy[i], qz[i], axis)
        d = d / np.linalg.norm(d)

        xs.append(x[i])
        ys.append(y[i])
        zs.append(z[i])
        us.append(d[0])
        vs.append(d[1])
        ws.append(d[2])

    ax.quiver(
        xs,
        ys,
        zs,
        us,
        vs,
        ws,
        length=scale,
        normalize=True,
        color="red",
        linewidth=1.0,
    )

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("End-effector path with orientation")
    ax.legend()


def plot_angular_speed(t, qw, qx, qy, qz):
    rotvecs = np.array(
        [quat_to_rotvec(qw[i], qx[i], qy[i], qz[i]) for i in range(len(t))]
    )

    dt = np.diff(t)
    drot = np.diff(rotvecs, axis=0)

    omega = np.linalg.norm(drot / dt[:, None], axis=1)

    plt.figure()
    plt.plot(t[1:], omega)
    plt.xlabel("time (s)")
    plt.ylabel("angular speed (rad/s)")
    plt.title("Angular speed vs time")
    plt.grid(True)


if __name__ == "__main__":
    t, x, y, z, qw, qx, qy, qz = load_traj_csv("../build/examples/trajectory.csv")

    plot_traj_3d_with_orientation(
        x,
        y,
        z,
        qw,
        qx,
        qy,
        qz,
        stride=1,  # tune for density
        axis=np.array([0, 0, 1]),  # EE Z-axis
        scale=0.05,
    )
    # plot_xyz_vs_time(t, x, y, z)
    #
    # plot_quaternion_vs_time(t, qw, qx, qy, qz)
    # plot_rotation_vector_vs_time(t, qw, qx, qy, qz)
    # plot_angular_speed(t, qw, qx, qy, qz)

    plt.show()
