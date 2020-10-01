import numpy as np
from scipy import signal

from geometry_msgs.msg import Quaternion
from mushr_rhc_ros.msg import SimpleTrajectory
import tf.transformations


def a2q(a):
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, a))


def config2simpletraj(config, pusher_measures):
    # car pose and block pose.
    x, y, t = config[0]

    base2pushx = float(pusher_measures["x_pusher_pos_base_link"])
    airgap = float(pusher_measures["air_gap"])
    bsl = float(pusher_measures["block_side_len"])

    base_link_to_push = base2pushx + bsl / 2.0 + airgap
    s, c = np.sin(t), np.cos(t)

    car_start = [float((-base_link_to_push * c) + x),
                 float((-base_link_to_push * s) + y),
                 float(t)]

    block_start = [float(x), float(y), float(t)]

    return carblockgoal2simpletraj(car_start, block_start, config, pusher_measures)


def carblockgoal2simpletraj(cs, bs, config, pusher_measures):
    traj = SimpleTrajectory()
    bsl = float(pusher_measures["block_side_len"])

    x, y, t = bs
    block_start = [float(x), float(y), float(bsl / 2.0), float(t)]

    traj.block_pose.position.x = x
    traj.block_pose.position.y = y
    traj.block_pose.position.z = bsl / 2.0
    traj.block_pose.orientation = a2q(t)

    x, y, t = cs
    car_start = [float(x), float(y), 0, float(t)]

    traj.car_pose.position.x = x
    traj.car_pose.position.y = y
    traj.car_pose.orientation = a2q(t)

    for c in config:
        traj.xs.append(c[0])
        traj.ys.append(c[1])
        traj.thetas.append(c[2])

    points = []
    for c in config:
        points.append(map(float, c))

    return traj, dict(car_start=car_start,
                      block_start=block_start,
                      points=points)


def saw():
    t = np.linspace(-4, 4, 50)
    saw = signal.sawtooth(0.5 * np.pi * t)
    configs = [[x, y, 0] for (x, y) in zip(t, saw)]
    return configs


def wave():
    ts = np.linspace(-5, 5, 50)
    period = 0.8
    ys = np.sin(period * ts)
    theta = np.cos(period * ts)
    configs = [[x, y, _theta] for (x, y, _theta) in zip(ts, ys, theta)]
    return configs


def circle():
    waypoint_sep = 0.1
    radius = 2.5
    center = [0, radius]
    num_points = int((2 * radius * np.pi) / waypoint_sep)
    thetas = np.linspace(-1 * np.pi / 2, 2 * np.pi - (np.pi / 2), num_points)
    poses = [[radius * np.cos(theta) + center[0], radius * np.sin(theta) + center[1], theta + (np.pi / 2)] for theta in thetas]
    return poses


def left_turn(turn_rad, pathlen=4.0):
    waypoint_sep = 0.1
    turn_radius = turn_rad
    straight_len = 0.0
    turn_center = [straight_len, turn_radius]
    straight_xs = np.linspace(0, straight_len, int(straight_len / waypoint_sep))
    straight_poses = [[x, 0, 0] for x in straight_xs]

    circumf = 2 * np.pi * turn_rad
    if pathlen > circumf:
        circumf = pathlen

    num_turn_points = int(pathlen / waypoint_sep)  # int((turn_radius * np.pi * 0.5) / waypoint_sep)
    thetas = -np.pi / 2 + np.linspace(0, (pathlen / circumf) * 2 * np.pi, num_turn_points)

    turn_poses = [[turn_radius * np.cos(theta) + turn_center[0],
                   turn_radius * np.sin(theta) + turn_center[1],
                   theta + (np.pi / 2)] for theta in thetas]
    poses = straight_poses + turn_poses
    return poses[:min(int(pathlen / waypoint_sep), len(poses))]


def left_kink(turn_rad, pathlen=4.0, circumf=np.pi * 3.0 / 8.0):
    waypoint_sep = 0.1
    straight_len = 1.0
    turn1_center = [straight_len, turn_rad]

    straight_xs = np.linspace(0, straight_len, int(straight_len / waypoint_sep))
    straight_poses = [[x, 0, 0] for x in straight_xs]

    num_turn1_points = int((circumf * turn_rad) / waypoint_sep)
    turn1_thetas = -np.pi / 2.0 + np.linspace(0, circumf, num_turn1_points, endpoint=True)
    turn1_poses = [[turn_rad * np.cos(theta) + turn1_center[0], turn_rad * np.sin(theta) + turn1_center[1], theta + (np.pi / 2)] for theta in turn1_thetas]

    cur_pathlen = straight_len + circumf * turn_rad

    if cur_pathlen < pathlen:
        kink_xs = np.linspace(0, pathlen - cur_pathlen, int((pathlen - cur_pathlen) / waypoint_sep))
        kink = [[x * np.cos(turn1_poses[-1][2]) + turn1_poses[-1][0],
                 x * np.sin(turn1_poses[-1][2]) + turn1_poses[-1][1],
                 turn1_poses[-1][2]] for i, x in enumerate(kink_xs)]
    else:
        kink = []

    poses = straight_poses + turn1_poses + kink
    return poses[:min(int(pathlen / waypoint_sep), len(poses))]


def right_turn(turn_rad, pathlen=4.0):
    waypoint_sep = 0.1
    turn_radius = turn_rad
    straight_len = 0.0
    turn_center = [straight_len, -turn_radius]
    straight_xs = np.linspace(0, straight_len, int(straight_len / waypoint_sep))
    straight_poses = [[x, 0, 0] for x in straight_xs]

    circumf = 2 * np.pi * turn_rad
    if pathlen > circumf:
        circumf = pathlen

    num_turn_points = int(pathlen / waypoint_sep)  # int((turn_radius * np.pi * 0.5) / waypoint_sep)
    thetas = np.pi / 2 + np.linspace(0, -(pathlen / circumf) * 2 * np.pi, num_turn_points)

    turn_poses = [[turn_radius * np.cos(theta) + turn_center[0],
                   turn_radius * np.sin(theta) + turn_center[1],
                   theta - (np.pi / 2)] for theta in thetas]
    poses = straight_poses + turn_poses
    return poses[:min(int(pathlen / waypoint_sep), len(poses))]


def right_kink(turn_rad, pathlen=4.0, circumf=(np.pi * 3.0 / 8.0)):
    waypoint_sep = 0.1
    straight_len = 1.0
    turn1_center = [straight_len, -turn_rad]

    straight_xs = np.linspace(0, straight_len, int(straight_len / waypoint_sep))
    straight_poses = [[x, 0, 0] for x in straight_xs]

    num_turn1_points = int((circumf * turn_rad) / waypoint_sep)
    turn1_thetas = np.pi / 2 + np.linspace(0, -circumf, num_turn1_points, endpoint=True)
    turn1_poses = [[turn_rad * np.cos(theta) + turn1_center[0], turn_rad * np.sin(theta) + turn1_center[1], theta - (np.pi / 2)] for theta in turn1_thetas]

    cur_pathlen = straight_len + circumf * turn_rad

    if cur_pathlen < pathlen:
        kink_xs = np.linspace(0, pathlen - cur_pathlen, int((pathlen - cur_pathlen) / waypoint_sep))
        kink = [[x * np.cos(turn1_poses[-1][2]) + turn1_poses[-1][0],
                 x * np.sin(turn1_poses[-1][2]) + turn1_poses[-1][1],
                 turn1_poses[-1][2]] for i, x in enumerate(kink_xs)]
    else:
        kink = []

    poses = straight_poses + turn1_poses + kink
    return poses[:min(int(pathlen / waypoint_sep), len(poses))]


def straight_line(pathlen=4.0):
    waypoint_sep = 0.1
    straight_len = pathlen
    straight_xs = np.linspace(0, straight_len, int(straight_len / waypoint_sep))
    straight_poses = [[x, 0, 0] for x in straight_xs]
    return straight_poses


xs = [
    -0.335805535316,
    -0.650691986084,
    -0.808273911476,
    -1.62306058407,
    -2.22904491425,
    -2.11577129364,
    -2.14611577988,
    -2.15082144737,
    -2.59145474434,
    -2.98570728302,
    -3.99445772171,
    -4.53280162811,
    -5.08250045776,
    -5.34967660904,
    -5.481341362,
    -4.71273326874,
    -3.94519734383,
    -3.35369920731,
    -2.77183747292,
    -2.20369744301,
]

ys = [
    -0.404051423073,
    -0.160076498985,
    -0.166414380074,
    0.340170860291,
    0.624907672405,
    1.16461098194,
    2.01381158829,
    2.60687971115,
    2.97159957886,
    3.21234703064,
    2.94576716423,
    2.0039896965,
    1.20905208588,
    0.325662016869,
    -0.373188734055,
    -0.640047669411,
    -1.13770210743,
    -1.25276899338,
    -1.44814252853,
    -1.32985377312,
]


def real_traj():
    x_off = 4.98228740692
    y_off = 2.2  # 2.62920451164

    return [[x + x_off, y + y_off, 0] for x, y in zip(xs, ys)]
