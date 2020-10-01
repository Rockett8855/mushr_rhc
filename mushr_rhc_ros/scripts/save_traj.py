#!/usr/bin/env python
import pickle
import os
import yaml
import numpy as np

import trajgen


pathlen = 6.0
# RADS = [2.5, 4.0, 10.0]
TURN_RADS = np.array([1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 12.5, 15.0, 20.0, 50.0])

# kr = np.linspace(0.1, 5.0, num=5, endpoint=True)
# kc = np.pi * (3.0 / 8.0) + np.linspace(0, np.pi / 4.0, num=5, endpoint=True)
# KINK_RAD_CIRCUM = np.transpose([np.tile(kr, kc.shape[0]), np.repeat(kc, kr.shape[0])])

KINK_RAD_CIRCUM = [
    (1.0, np.pi * (3.0 / 8.0) + np.linspace(0, np.pi / 4.0, num=5, endpoint=True)),
    (2.0, np.pi * (3.0 / 8.0) + np.linspace(0, np.pi / 4.0, num=3, endpoint=True)),
    (3.0, np.pi * (3.0 / 8.0) + np.linspace(0, np.pi / 4.0, num=3, endpoint=True)),
]
KINK_RAD_CIRCUM = []

turns = [("left-turn", trajgen.left_turn), ("right-turn", trajgen.right_turn)]
kinks = [("left-kink", trajgen.left_kink), ("right-kink", trajgen.right_kink)]


def dump(folder, fname, trajpickle, trajyaml):
    with open("%s/pickle/%s.pickle" % (folder, fname), "wb") as f:
        pickle.dump(trajpickle, f)
    with open("%s/yaml/%s.yaml" % (folder, fname), "wb") as f:
        yaml.dump(trajyaml, f)


if __name__ == "__main__":
    pm_path = os.path.expanduser("~/catkin_ws/src/pixel_art/pixel_art_common/params/pusher_measures.yaml")
    with open(pm_path, "r") as f:
        pusher_measures = yaml.load(f)

    folder = "turns_only_may29"
    os.mkdir(folder)
    os.mkdir(folder + "/pickle")
    os.mkdir(folder + "/yaml")

    config = trajgen.straight_line(pathlen)
    trajpickle, trajyaml = trajgen.config2simpletraj(config, pusher_measures)

    for r in TURN_RADS:
        for fname, f in turns:
            config = f(r, pathlen=pathlen)
            trajpickle, trajyaml = trajgen.config2simpletraj(config, pusher_measures)

            dump(folder, "%s-%s" % (fname, r), trajpickle, trajyaml)

    for r, cs in KINK_RAD_CIRCUM:
        for fname, fn in kinks:
            for c in cs:
                config = fn(r, pathlen=pathlen, circumf=c)
                trajpickle, trajyaml = trajgen.config2simpletraj(config, pusher_measures)

                dump(folder, "%s-%s-%s" % (fname, r, c), trajpickle, trajyaml)
