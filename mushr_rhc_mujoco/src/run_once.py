#!/usr/bin/env python

# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

import torch
import argparse
import pickle
import os
import yaml
import mushr_mujoco_py
import numpy as np
import re

import tf
import rospy

import mushr_rhc.defaults.logger as logger
import mushr_rhc.defaults.parameters as parameters

import mushr_rhc
import mushr_rhc.cost
import mushr_rhc.model
import mushr_rhc.trajgen
import mushr_rhc.value
import mushr_rhc.worldrep


MAP = None


class MujocoPySim:
    def __init__(self, params, logger, dtype, **kwargs):
        self.K = params.get_int("K", default=62)
        self.T = params.get_int("T", default=21)
        self.NPOS = params.get_int("npos", default=3)

        time_horizon = mushr_rhc.utils.get_time_horizon(params)
        self.dt = time_horizon / self.T

        self.pystate = kwargs["pystate"]
        self.nprollout = np.zeros((self.K, self.T, self.NPOS))

    def rollout(self, state, trajs, rollouts):
        self.pystate.block_car_rollout("buddy", "block", self.dt, trajs, self.nprollout)
        rollouts[:, :, :] = torch.from_numpy(self.nprollout)

        # import matplotlib.pyplot as plt

        # print rollouts
        # for r in rollouts:
        #     plt.plot(r[:, 0], r[:, 1])
        #     plt.plot(r[:, 3], r[:, 4])
        # plt.show()


motion_models = {
    "kinematic": mushr_rhc.model.Kinematics,
    "mujoco": MujocoPySim,
    "gpr": mushr_rhc.model.GPR,
    "nearestneighbor": mushr_rhc.model.NearestNeighbor,
}

trajgens = {
    "tl": mushr_rhc.trajgen.TL,
    "dispersion": mushr_rhc.trajgen.Dispersion,
    "mppi": mushr_rhc.trajgen.MXPI,
    "mxpi": mushr_rhc.trajgen.MXPI,  # what i get for using multiple names...
    "block_push": mushr_rhc.trajgen.BlockPush,
    "gaussian": mushr_rhc.trajgen.Gaussian,
}

cost_functions = {
    "waypoints": mushr_rhc.cost.Waypoints,
    "block_push": mushr_rhc.cost.BlockPush,
    "block_ref_traj_means": mushr_rhc.cost.BlockRefTrajectory,
    "block_ref_traj_cov": mushr_rhc.cost.BlockRefTrajectoryCovariance,
    "block_ref_traj_complex": mushr_rhc.cost.BlockRefTrajectoryComplex,
}

value_functions = {"simpleknn": mushr_rhc.value.SimpleKNN}

world_reps = {"simple": mushr_rhc.worldrep.Simple}


def load_controller(params, logger, pystate, map_data):
    m = get_model(params, logger, pystate)
    tg = get_trajgen(params, logger)
    cf = get_cost_fn(params, logger, map_data)

    return mushr_rhc.MPC(params, logger, torch.FloatTensor, m, tg, cf)


def get_model(params, logger, pystate):
    mname = params.get_str("model_name", default="kinematic")
    if mname not in motion_models:
        logger.fatal("model '{}' is not valid".format(mname))

    return motion_models[mname](params, logger, torch.FloatTensor, pystate=pystate)


def get_trajgen(params, logger):
    tgname = params.get_str("trajgen_name", default="tl")
    if tgname not in trajgens:
        logger.fatal("trajgen '{}' is not valid".format(tgname))

    logger.warn("using trajgen '{}'".format(tgname))
    return trajgens[tgname](params, logger, torch.FloatTensor)


def get_cost_fn(params, logger, map_data):
    cfname = params.get_str("cost_fn_name", default="waypoints")
    if cfname not in cost_functions:
        logger.fatal("cost_fn '{}' is not valid".format(cfname))

    wrname = params.get_str("world_rep_name", default="simple")
    if wrname not in world_reps:
        logger.fatal("world_rep '{}' is not valid".format(wrname))

    logger.info("Waiting for map metadata")
    while map_data is None:
        rospy.sleep(0.1)
    logger.debug("Recieved map metadata")

    wr = world_reps[wrname](params, logger, torch.FloatTensor, map_data)

    vfname = params.get_str("value_fn_name", default="simpleknn")
    if vfname not in value_functions:
        logger.fatal("value_fn '{}' is not valid".format(vfname))

    vf = value_functions[vfname](
        params, logger, torch.FloatTensor, map_data, False
    )

    viz_rollouts_fn = None

    return cost_functions[cfname](
        params, logger, torch.FloatTensor, map_data, wr, vf, viz_rollouts_fn
    )


def read_pgm(filename, byteorder='>'):
    """Return image data from a raw PGM file as numpy array.

    Format specification: http://netpbm.sourceforge.net/doc/pgm.html

    """
    with open(filename, 'rb') as f:
        buf = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buf).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)

    arr = np.frombuffer(buf,
                        dtype='u1' if int(maxval) < 256 else byteorder + 'u2',
                        count=int(width) * int(height),
                        offset=len(header)).reshape((int(height), int(width)))

    return width, height, arr


def map_metadata(map_info_file, map_data_file):
    with open(map_info_file, "r") as f:
        map_info = yaml.load(f)

    width, height, MAP = read_pgm(map_data_file)

    return mushr_rhc.MapInfo(
        name="empty",
        resolution=map_info["resolution"],
        origin_x=map_info["origin"][0],
        origin_y=map_info["origin"][1],
        orientation_angle=map_info["origin"][2],
        width=width,
        height=height,
        get_map_data=lambda: MAP,
    )


def quat2yaw(w, x, y, z):
    """Convert a quaternion _message_ into an angle in radians.
    The angle represents the yaw.
    This is not just the z component of the quaternion."""
    _, _, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
    return yaw


def rospose_to_simstate(pose):
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z
    qw = pose.orientation.w
    qx = pose.orientation.x
    qy = pose.orientation.y
    qz = pose.orientation.z
    return (x, y, z, qw, qx, qy, qz)


def simstate_to_thisstate((simtime, s)):
    bu = s['buddy']
    bl = s['block']
    return simtime, torch.FloatTensor([bu[0], bu[1], quat2yaw(*bu[3:]), bl[0], bl[1], quat2yaw(*bl[3:])])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("traj_file")
    parser.add_argument("run_out")
    parser.add_argument("config_file")
    parser.add_argument("--sim_timeout", type=float, default=10.0)

    args = parser.parse_args()

    params_yaml = {}
    pusher_measures_file = os.path.expanduser("~/catkin_ws/src/pixel_art/pixel_art_common/params/pusher_measures.yaml")
    with open(pusher_measures_file) as f:
        pusher_measures_yaml = yaml.load(f)
        for k, v in pusher_measures_yaml.items():
            params_yaml["/pusher_measures/" + k] = v

    with open(args.config_file, "rb") as f:
        params_yaml.update(yaml.load(f))

    params = parameters.DictParams(params_yaml)
    logger = logger.StdLog()

    mj_key = os.path.expanduser("~/.mujoco/mjkey.txt")
    model_file_path = os.path.expanduser("~/catkin_ws/src/mushr_mujoco_ros/models/block.xml")
    config_file = os.path.expanduser("~/catkin_ws/src/mushr_mujoco_ros/config/block.yaml")
    map_info_file = os.path.expanduser("~/catkin_ws/src/mushr_mujoco_ros/maps/empty.yaml")
    map_data_file = os.path.expanduser("~/catkin_ws/src/mushr_mujoco_ros/maps/empty.pgm")
    viz = False
    record = False
    record_camera = ""
    record_out_file = ""

    print args.traj_file, args.run_out, args.config_file

    with open(args.traj_file, "rb") as f:
        msg = pickle.load(f)

    traj = torch.empty((len(msg.xs), 3)).type(torch.FloatTensor)
    for i in range(len(msg.xs)):
        traj[i, 0] = msg.xs[i]
        traj[i, 1] = msg.ys[i]
        traj[i, 2] = msg.thetas[i]

    run_out = {}

    run_out["poses_and_times"] = []
    run_out["failure"] = True
    run_out["traj"] = traj

    sim = mushr_mujoco_py.PySimState(mj_key, model_file_path, config_file, viz, record,
                                     record_camera, record_out_file)
    simtime, state = simstate_to_thisstate(
        sim.reset({"block": rospose_to_simstate(msg.block_pose),
                   "buddy": rospose_to_simstate(msg.car_pose)})
    )
    map_data = map_metadata(map_info_file, map_data_file)
    rhctrl = load_controller(params, logger, sim, map_data)

    rhctrl.set_trajectory(traj)

    print args.sim_timeout, type(args.sim_timeout)
    while simtime < args.sim_timeout:
        if torch.norm(state[:2] - state[3:5]) > 0.5:
            run_out["failed"] = True
            break

        if rhctrl.cost.at_goal(state):
            run_out["failed"] = False
            break

        run_out["poses_and_times"].append((simtime, state))
        next_traj, rollout = rhctrl.step(state)
        simtime, state = simstate_to_thisstate(sim.step({"buddy": (next_traj[0, 0], next_traj[0, 1])}))

    if simtime >= args.sim_timeout:
        run_out["failed"] = True

    print "simtime", simtime, "sim_timeout", args.sim_timeout
    print "START Dumping to", args.run_out
    with open(args.run_out, "wb") as f:
        pickle.dump(run_out, f)
    print "DONE  Dumping to", args.run_out
