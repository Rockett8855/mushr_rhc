# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

import torch
import mushr_rhc.utils as utils
import threading
import math

# send marker
import rospy
import tf.transformations
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion


def a2q(a):
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, a))
# end send


class BlockRefTrajectory:
    def __init__(
        self, params, logger, dtype, map, world_rep, value_fn, viz_rollouts_fn
    ):
        self.params = params
        self.logger = logger
        self.dtype = dtype
        self.map = map

        self.world_rep = world_rep
        self.value_fn = value_fn

        self.viz_rollouts_fn = viz_rollouts_fn

        self.ROS = params.get_bool("rosversion", global_=True, default=False)
        if self.ROS:
            self.DEBUG = rospy.Publisher("~current_marker", Marker, queue_size=1)
            self.BLOCKARRAY = rospy.Publisher("~block_viz", MarkerArray, queue_size=1)

        self.reset()

    def reset(self):
        self.T = self.params.get_int("T", default=15)
        self.K = self.params.get_int("K", default=62)
        self.NPOS = self.params.get_int("npos", default=3)

        print self.T, self.K

        self.dist_horizon = utils.get_distance_horizon(self.params)
        self.traj_lock = threading.RLock()
        with self.traj_lock:
            self.goal = None
        self.goal_threshold = self.params.get_float("xy_threshold", default=0.2)

        horizon = utils.get_distance_horizon(self.params)
        self.waypoint_lookahead = 0.2  # self.dist_horizon
        self.waypoint_idx_lookahead = int(math.ceil(horizon / 0.5)) + 1  # 3

        self.dist_w = self.params.get_float("cost_fn/block_ref_traj/dist_w", default=.75)
        self.manip_w = self.params.get_float("cost_fn/block_ref_traj/manip_w", default=4.5)
        self.contact_w = self.params.get_float("cost_fn/block_ref_traj/contact_w", default=100.0)

        # self.decay = torch.exp(-torch.arange(0, self.T).type(self.dtype))
        self.decay = torch.ones(self.T,)

        self.world_rep.reset()

        print "dist_w", self.dist_w, "manip_w", self.manip_w, "contact_w", self.contact_w

    def apply(self, poses, (cov, manip, contact)):
        """
        Args:
        poses [(K, T, 3) tensor] -- Rollout of T positions
        goal  [(3,) tensor]: Goal position in "world" mode

        Returns:
        [(K,) tensor] costs for each K paths
        """
        assert poses.size() == (self.K, self.T, self.NPOS)

        # how many rollout steps should we look at?
        hidx = int(self.T * min(1.0, self.dist_to_goal(poses[0, 0]) / (self.dist_horizon)))

        with self.traj_lock:
            waypoint = self.get_waypoint(poses[0, 0, 3:5])

        # print waypoint, self.old_ref_idx
        # dist = torch.norm(poses[:, self.T - 1, 3:5] - waypoint[:2], dim=1).mul_(self.dist_w)

        all_dist = torch.norm(poses[:, :hidx, 3:5] - waypoint[:2], dim=2)
        traj_dists = torch.sum(all_dist, dim=1).div(hidx).mul_(self.dist_w)

        # # TRIED TO USE THIS FOR KEEPING THE BLOCK CENTERED
        car_block_1 = poses[:, :hidx, 3:5] - poses[:, :hidx, :2]
        s = torch.sin(-poses[:, :hidx, 2])
        c = torch.cos(-poses[:, :hidx, 2])
        block_car_y = car_block_1[:, :, 0] * s + car_block_1[:, :, 1] * c

        block_car_y.abs_()
        manipulability = torch.matmul(block_car_y, self.decay[:hidx]).div_(hidx).mul_(self.manip_w)

        ############################
        # CALCULATE MANIPULABILITY #
        ############################
        # summanip = torch.sum(manip[:, 1:hidx], dim=1).div_(-(hidx - 1)).mul_(self.manip_w)

        result = traj_dists.clone()
        result.add_(manipulability)
        if contact is None:
            contact = torch.isclose(poses[:, hidx - 1, 3:5], poses[:, hidx - 2, :2])
            contact = (~(contact[:, 0] & contact[:, 1])).type(self.dtype)

        result.add_(self.contact_w * (1 - contact))
        # result.add_(summanip)

        if self.viz_rollouts_fn:
            self.viz_rollouts_fn(
                result, poses, traj_dists=traj_dists, manipulability=manipulability, contact=contact
            )

            if self.ROS:
                ma = MarkerArray()
                for i, p in enumerate(poses[:, self.T - 1, 3:6]):
                    m = Marker()
                    m.header.frame_id = "map"
                    m.id = i
                    m.type = 1
                    m.pose.position.x = p[0]
                    m.pose.position.y = p[1]
                    m.pose.orientation = a2q(p[2])
                    m.scale.x = 0.1
                    m.scale.y = 0.1
                    m.scale.z = 0.05
                    m.color.r = float(i) / self.K
                    m.color.a = 1.0
                    ma.markers.append(m)
                self.BLOCKARRAY.publish(ma)

        return result, False

    def get_waypoint(self, state):
        dists = torch.norm(self.traj[:, :2] - state[:2], dim=1)
        idx = dists.argmin()
        idx += self.waypoint_idx_lookahead
        idx = min(idx, len(self.traj) - 1)
        return self.traj[idx]

    def set_trajectory(self, traj):
        """
        Args:
        goal [(3,) tensor] -- Goal in "world" coordinates
        """
        with self.traj_lock:
            self.traj = traj
            return True

    def dist_to_goal(self, state):
        with self.traj_lock:
            if self.traj is None:
                return False
            return self.traj[-1, :2].dist(state[3:5])

    def at_goal(self, state):
        """
        Args:
        state [(3,) tensor] -- Current position in "world" coordinates
        """
        with self.traj_lock:
            if self.traj is None:
                return False
            return self.dist_to_goal(state) < self.goal_threshold

    def get_desired_speed(self, desired_speed, state):
        return desired_speed
