#!/usr/bin/env python
import pickle
import os
import yaml
import numpy as np

import trajgen


pathlen = 6.0
TURN_RADS = []

# kr = np.linspace(0.1, 5.0, num=5, endpoint=True)
# kc = np.pi * (3.0 / 8.0) + np.linspace(0, np.pi / 4.0, num=5, endpoint=True)
# KINK_RAD_CIRCUM = np.transpose([np.tile(kr, kc.shape[0]), np.repeat(kc, kr.shape[0])])

KINK_RAD_CIRCUM = [
    (1.0, np.pi * (3.0 / 8.0) + np.linspace(0, np.pi / 2.0, num=5, endpoint=True)),
    (1.25, np.pi * (3.0 / 8.0) + np.linspace(0, np.pi / 2.0, num=5, endpoint=True)),
    (1.5, np.pi * (3.0 / 8.0) + np.linspace(0, np.pi / 2.0, num=5, endpoint=True)),
    (1.75, np.pi * (3.0 / 8.0) + np.linspace(0, np.pi / 2.0, num=5, endpoint=True)),
]

turns = [("left-turn", trajgen.left_turn), ("right-turn", trajgen.right_turn)]
kinks = [("left-kink", trajgen.left_kink), ("right-kink", trajgen.right_kink)]

if __name__ == "__main__":
    pm_path = os.path.expanduser("~/catkin_ws/src/pixel_art/pixel_art_common/params/pusher_measures.yaml")
    with open(pm_path, "r") as f:
        pusher_measures = yaml.load(f)

    folder = "kinks"

    config = trajgen.straight_line(pathlen)
    traj = trajgen.config2simpletraj(config, pusher_measures)
    with open("%s/straight.pickle" % folder, "wb") as f:
        pickle.dump(traj, f)

    for r in TURN_RADS:
        for fname, f in turns:
            config = f(r, pathlen=pathlen)
            traj = trajgen.config2simpletraj(config, pusher_measures)

            with open("%s/%s-%s.pickle" % (folder, fname, r), "wb") as f:
                pickle.dump(traj, f)

    for r, cs in KINK_RAD_CIRCUM:
        for fname, fn in kinks:
            for c in cs:
                config = fn(r, pathlen=pathlen, circumf=c)
                traj = trajgen.config2simpletraj(config, pusher_measures)

                with open("%s/%s-%s-%s.pickle" % (folder, fname, r, c), "wb") as f:
                    pickle.dump(traj, f)
