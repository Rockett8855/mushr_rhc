#!/usr/bin/env python
import pickle
import os
import yaml
import numpy as np

import trajgen

outer_rad = 3.5
inner_rad = 1.0
N = 4
angles = np.linspace(0, 2 * np.pi, num=N, endpoint=False)

OUTER = zip(outer_rad * np.cos(angles), outer_rad * np.sin(angles), np.zeros(N))
INNER = zip(inner_rad * np.cos(angles), outer_rad * np.sin(angles), np.zeros(N))


def dump(folder, fname, trajpickle, trajyaml):
    with open("%s/pickle/%s.pickle" % (folder, fname), "wb") as f:
        pickle.dump(trajpickle, f)
    with open("%s/yaml/%s.yaml" % (folder, fname), "wb") as f:
        yaml.dump(trajyaml, f)


if __name__ == "__main__":
    pm_path = os.path.expanduser("~/catkin_ws/src/pixel_art/pixel_art_common/params/pusher_measures.yaml")
    with open(pm_path, "r") as f:
        pusher_measures = yaml.load(f)

    folder = "complex"
    os.mkdir(folder)
    os.mkdir(folder + "/pickle")
    os.mkdir(folder + "/yaml")

    j = 0
    for o in OUTER:
        for i in INNER:
            zero = (0.0, 0.0, 0.0)
            trajpickle, trajyaml = trajgen.carblockgoal2simpletraj(
                cs=zero, bs=o, config=[o, i], pusher_measures=pusher_measures)

            j += 1
            dump(folder, "%d" % j, trajpickle, trajyaml)

            trajpickle, trajyaml = trajgen.carblockgoal2simpletraj(
                cs=zero, bs=i, config=[i, o], pusher_measures=pusher_measures)

            j += 1
            dump(folder, "%d" % j, trajpickle, trajyaml)
