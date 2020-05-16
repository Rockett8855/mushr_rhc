#!/usr/bin/env python
import os
import yaml

from mushr_rhc_ros.msg import SimpleTrajectory
import time
import trajgen
import rospy


plans = {
    'circle': trajgen.circle,
    'wave': trajgen.wave,
    'striaght_line': trajgen.straight_line,
    'left-10': lambda: trajgen.left_turn(10.0, 6.0),
    'left-4': lambda: trajgen.left_turn(4.0, 6.0),
    'left-2.5': lambda: trajgen.left_turn(2.5, 6.0),
    'right-10': lambda: trajgen.right_turn(10.0, 6.0),
    'right-4': lambda: trajgen.right_turn(4.0, 6.0),
    'right-2.5': lambda: trajgen.right_turn(2.5, 6.0),
    'left-kink-1.0': lambda: trajgen.left_kink(1.0, 6.),
    'left-kink-3.0': lambda: trajgen.left_kink(3.0, 6.),
    'left-kink-5.0': lambda: trajgen.left_kink(5.0, 6.),
    'right-kink-1.0': lambda: trajgen.right_kink(1.0, 6.),
    'right-kink-2.0': lambda: trajgen.right_kink(2.0, 6.),
    'right-kink-5.0': lambda: trajgen.right_kink(5.0, 6.),
    'right-kink-4.428': lambda: trajgen.right_kink(4.428571428571429, 6.),
    'real': trajgen.real_traj,
}


def get_plan():
    print "Which plan would you like to generate? "
    plan_names = plans.keys()
    for i, name in enumerate(plan_names):
        print "{} ({})".format(name, i)
    index = int(raw_input("num: "))
    if index >= len(plan_names):
        print "Wrong number. Exiting."
        exit()
    return plans[plan_names[index]]()


if __name__ == "__main__":
    rospy.init_node("controller_runner")
    config = get_plan()

    # h = Header()
    # h.stamp = rospy.Time.now()

    # desired_speed = 2.0
    # ramp_percent = 0.1
    # ramp_up = np.linspace(0.0, desired_speed, int(ramp_percent * len(configs)))
    # ramp_down = np.linspace(desired_speed, 0.3, int(ramp_percent * len(configs)))
    # speeds = np.zeros(len(configs))
    # speeds[:] = desired_speed
    # speeds[0:len(ramp_up)] = ramp_up
    # speeds[-len(ramp_down):] = ramp_down

    pm_path = os.path.expanduser("~/catkin_ws/src/pixel_art/pixel_art_common/params/pusher_measures.yaml")
    with open(pm_path, "r") as f:
        pusher_measures = yaml.load(f)
    traj = trajgen.config2simpletraj(config, pusher_measures)

    p = rospy.Publisher("/rhcontroller/trajectory", SimpleTrajectory, queue_size=1)
    time.sleep(1)

    print "Sending path..."
    p.publish(traj)
    print "Controller started."
