#!/usr/bin/env python

# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

import cProfile
import os
import signal
import threading
import torch

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PoseStamped, Vector3Stamped
from std_msgs.msg import ColorRGBA, Empty
from std_srvs.srv import Empty as SrvEmpty
from visualization_msgs.msg import Marker, MarkerArray
from mushr_rhc_ros.msg import SimpleTrajectory

from mushr_mujoco_ros.srv import Step, GetState, Reset
from mushr_mujoco_ros.msg import BodyStateArray

import mushr_rhc.defaults.logger as logger
import mushr_rhc.defaults.parameters as parameters
import rhcbase
import rhctensor
import utils


class RHCBlockBuddy(rhcbase.RHCBase):
    def __init__(self, dtype, params, logger, name):
        rospy.init_node(name, anonymous=True, disable_signals=True)

        super(RHCBlockBuddy, self).__init__(dtype, params, logger)

        self.reset_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self._state = None

        self.cur_rollout = self.cur_rollout_ip = None
        self.traj_pub_lock = threading.Lock()

        self.goal_event = threading.Event()
        self.map_metadata_event = threading.Event()
        self.ready_event = threading.Event()
        self.events = [self.goal_event, self.map_metadata_event, self.ready_event]
        self.run = True

        self.send_start_pose = self.params.get_bool("send_start_pose", default=True)
        self.do_profile = self.params.get_bool("profile", default=False)
        self.NPOS = self.params.get_int("npos", default=6)

    def start_profile(self):
        if self.do_profile:
            self.logger.warn("Running with profiling")
            self.pr = cProfile.Profile()
            self.pr.enable()

    def end_profile(self):
        if self.do_profile:
            self.pr.disable()
            self.pr.dump_stats(os.path.expanduser("~/mushr_rhc_stats.prof"))

    def start(self):
        self.logger.info("Starting RHController")
        self.start_profile()
        self.setup_pub_sub()
        self.rhctrl = self.load_controller()
        self.T = self.params.get_int("T")

        self.ready_event.set()

        rate = rospy.Rate(60)
        self.logger.info("Initialized")

        simstate = self.get_sim_state()
        self.set_state_from_block_state(simstate.body_state)
        self.block_state_pub.publish(simstate.body_state)

        while not rospy.is_shutdown() and self.run:
            # When running experiments, this will end the experiment when the block and car are
            # more than .5 meters away
            # if self.state is not None and torch.norm(self.state[:2] - self.state[3:5]) > 0.5:
            #     self.expr_failed.publish(Empty())
            #     self.goal_event.clear()

            next_traj, rollout = self.run_loop(self.state)

            # rospy.loginfo_throttle(1, "next_traj is none %s, state %s" % (next_traj is None, self.state))
            with self.traj_pub_lock:
                if rollout is not None:
                    self.cur_rollout = rollout.clone()
                    self.cur_rollout_ip = self.state[:2]

            if next_traj is not None:
                ctrlmsg = self.get_next_traj(next_traj, rollout)
                # rospy.loginfo_throttle(1, "before step")
                res = self.step(ctrlmsg)
                # rospy.loginfo_throttle(1, "after step")

                self.set_state_from_block_state(res.body_state)
                self.block_state_pub.publish(res.body_state)

                # For experiments. If the car is at the goal, notify the
                # experiment tool
                if self.rhctrl.cost.at_goal(self.state):
                    self.expr_at_goal.publish(Empty())
                    self.goal_event.clear()
            rate.sleep()

        self.end_profile()

    def run_loop(self, ip):
        self.goal_event.wait()
        if rospy.is_shutdown() or ip is None:
            return None, None
        with self.reset_lock:
            # If a reset is initialed after the goal_event was set, the goal
            # will be cleared. So we have to have another goal check here.
            if not self.goal_event.is_set():
                return None, None
            if ip is not None:
                return self.rhctrl.step(ip)
            self.logger.err("Shouldn't get here: run_loop")

    def shutdown(self, signum, frame):
        rospy.signal_shutdown("SIGINT recieved")
        self.run = False
        for ev in self.events:
            ev.set()

    def setup_pub_sub(self):
        rospy.Service("~reset/soft", SrvEmpty, self.srv_reset_soft)
        rospy.Service("~reset/hard", SrvEmpty, self.srv_reset_hard)

        rospy.loginfo("Waiting for /mushr_mujoco_ros/step")
        rospy.wait_for_service("/mushr_mujoco_ros/step")
        self.step = rospy.ServiceProxy("/mushr_mujoco_ros/step", Step, persistent=True)

        rospy.loginfo("Waiting for /mushr_mujoco_ros/state")
        rospy.wait_for_service("/mushr_mujoco_ros/state")
        self.get_sim_state = rospy.ServiceProxy("/mushr_mujoco_ros/state", GetState, persistent=True)

        rospy.loginfo("Waiting for /mushr_mujoco_ros/reset")
        rospy.wait_for_service("/mushr_mujoco_ros/reset")
        self.reset_sim = rospy.ServiceProxy("/mushr_mujoco_ros/reset", Reset, persistent=True)

        rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.cb_goal, queue_size=1
        )

        rospy.Subscriber(
            "~trajectory", SimpleTrajectory, self.cb_trajectory, queue_size=1
        )

        rospy.Subscriber(
            rospy.get_param("~inferred_pose_t"),
            PoseStamped,
            self.cb_pose,
            queue_size=10,
        )

        rospy.Subscriber(
            rospy.get_param("~block_pose_t"),
            PoseStamped,
            self.cb_block_pose,
            queue_size=10,
        )

        if self.params.get_bool("velocity_in_state"):
            if self.NPOS < 7:
                self.logger.warn("Want to add velocity to state, but NPOS is only %d" % self.NPOS)
            rospy.Subscriber(
                rospy.get_param("~car_velocity_t"),
                Vector3Stamped,
                self.cb_car_velocity,
                queue_size=10,
            )

        traj_chosen_t = self.params.get_str("traj_chosen_topic", default="~traj_chosen")
        self.traj_chosen_pub = rospy.Publisher(traj_chosen_t, Marker, queue_size=10)
        self.viz_traj_pub = rospy.Publisher("~ref_traj", MarkerArray, queue_size=10)

        self.block_state_pub = rospy.Publisher("~body_state", BodyStateArray, queue_size=10)
        # For the experiment framework, need indicators to listen on
        self.expr_at_goal = rospy.Publisher("/experiment_tool/finished", Empty, queue_size=1)
        self.expr_failed = rospy.Publisher("/experiment_tool/failed", Empty, queue_size=1)

    def srv_reset_hard(self, msg):
        """
        Hard reset does a complete reload of the controller
        """
        rospy.loginfo("Start hard reset")
        self.reset_lock.acquire()
        self.load_controller()
        self.goal_event.clear()
        self.reset_lock.release()
        rospy.loginfo("End hard reset")
        return []

    def srv_reset_soft(self, msg):
        """
        Soft reset only resets soft state (like tensors). No dependencies or maps
        are reloaded
        """
        rospy.loginfo("Start soft reset")
        self.reset_lock.acquire()
        self.rhctrl.reset()
        self.goal_event.clear()
        self.reset_lock.release()
        rospy.loginfo("End soft reset")
        return []

    def cb_trajectory(self, msg):
        traj = torch.empty((len(msg.xs), 3)).type(self.dtype)
        for i in range(len(msg.xs)):
            traj[i, 0] = msg.xs[i]
            traj[i, 1] = msg.ys[i]
            traj[i, 2] = msg.thetas[i]

        with self.reset_lock:
            self.rhctrl.reset()
            self.goal_event.clear()

        self.viz_trajectory(msg)

        rospy.loginfo("Reset simulator")
        res = self.reset_sim(["block", "buddy"], [msg.block_pose, msg.car_pose])
        self.set_state_from_block_state(res.body_state)
        self.block_state_pub.publish(res.body_state)

        if not self.rhctrl.set_trajectory(traj):
            self.logger.err("Couldn't set reference trajectory")
            return
        else:
            self.logger.info("Trajectory set")

        self.goal_event.set()

    def viz_trajectory(self, traj):
        ma = MarkerArray()
        for i in range(len(traj.xs)):
            m = Marker()
            m.header.frame_id = "map"
            m.id = i
            m.action = m.ADD
            m.type = m.ARROW
            m.pose.position.x = traj.xs[i]
            m.pose.position.y = traj.ys[i]

            m.scale.x = 0.05
            m.scale.y = 0.05
            m.scale.z = 0.01

            m.color.r = 0.0
            m.color.g = 0.5
            m.color.b = 0.0
            m.color.a = 1.0
            m.pose.orientation = utils.angle_to_rosquaternion(traj.thetas[i])
            ma.markers.append(m)

        self.viz_traj_pub.publish(ma)

    def cb_goal(self, msg):
        goal = self.dtype(utils.rospose_to_posetup(msg.pose))
        self.ready_event.wait()
        if not self.rhctrl.set_goal(goal):
            self.logger.err("That goal is unreachable, please choose another")
            return
        else:
            self.logger.info("Goal set")
        self.goal_event.set()

    def cb_block_pose(self, msg):
        self.set_block_pose(self.dtype(utils.rospose_to_posetup(msg.pose)))

    def cb_pose(self, msg):
        self.set_inferred_pose(self.dtype(utils.rospose_to_posetup(msg.pose)))

        if self.cur_rollout is not None and self.cur_rollout_ip is not None:
            m = Marker()
            m.header.frame_id = "map"
            m.type = m.LINE_STRIP
            m.action = m.ADD
            with self.traj_pub_lock:
                pts = (
                    self.cur_rollout[:, :2] - self.cur_rollout_ip[:2]
                ) + self.state[:2]

            m.points = map(lambda xy: Point(x=xy[0], y=xy[1]), pts)

            r, g, b = 0x36, 0xCD, 0xC4
            m.colors = [ColorRGBA(r=r / 255.0, g=g / 255.0, b=b / 255.0, a=0.7)] * len(
                m.points
            )
            m.scale.x = 0.05
            self.traj_chosen_pub.publish(m)

    def cb_car_velocity(self, msg):
        with self.state_lock:
            if self._state is None:
                self._state = self.dtype(self.NPOS)
            self._state[6] = torch.sqrt(self.dtype([msg.vector.x ** 2 + msg.vector.y ** 2 + msg.vector.z ** 2]))

    def get_next_traj(self, traj, rollout):
        assert traj.size() == (self.T, 2)

        ctrl = traj[0]
        ctrlmsg = AckermannDriveStamped()
        ctrlmsg.header.stamp = rospy.Time.now()
        ctrlmsg.drive.speed = ctrl[0]
        ctrlmsg.drive.steering_angle = ctrl[1]
        return ctrlmsg

    def set_inferred_pose(self, ip):
        with self.state_lock:
            if self._state is None:
                self._state = self.dtype(self.NPOS)
            self._state[:3] = ip

    def set_block_pose(self, ip):
        with self.state_lock:
            if self._state is None:
                self._state = self.dtype(self.NPOS)
            self._state[3:6] = ip

    def set_state_from_block_state(self, bs):
        for st in bs.states:
            if st.name == "buddy":
                self.set_inferred_pose(self.dtype(utils.rospose_to_posetup(st.pose)))
            if st.name == "block":
                self.set_block_pose(self.dtype(utils.rospose_to_posetup(st.pose)))

    @property
    def state(self):
        with self.state_lock:
            return self._state


if __name__ == "__main__":
    params = parameters.RosParams()
    logger = logger.RosLog()
    node = RHCBlockBuddy(rhctensor.float_tensor(), params, logger, "rhcontroller")

    signal.signal(signal.SIGINT, node.shutdown)
    rhc = threading.Thread(target=node.start)
    rhc.start()

    # wait for a signal to shutdown
    while node.run:
        signal.pause()

    rhc.join()
