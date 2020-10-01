import matplotlib.pyplot as plt
import pickle
import numpy as np
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("outfile")
    args = parser.parse_args()

    with open(args.outfile, "rb") as f:
        trial = pickle.load(f)

    print "failed?", trial["failed"]

    print trial["controls"][:10]
    car = np.zeros((len(trial["poses_and_times"]), 3))
    block = np.zeros((len(trial["poses_and_times"]), 3))

    for i, (t, p) in enumerate(trial["poses_and_times"]):
        car[i] = p[:3]
        block[i] = p[3:]

    plt.scatter(trial['traj'][:, 0], trial['traj'][:, 1])
    plt.plot(car[:, 0], car[:, 1], label="car")
    plt.plot(block[:, 0], block[:, 1], label="block")
    plt.legend()
    plt.axis("equal")
    plt.show()
