#!/usr/bin/env python

import argparse
import datetime as dt
import os
import shutil
import threading
import subprocess32 as subprocess
import time

# Timeout in seconds to run the simulation
RUN_TIMEOUT = 60


class Trial:
    def __init__(self, tname, desc_path, params_file):
        self.tname = tname
        self.descpath = desc_path
        self.params_file = params_file

    def __repr__(self):
        return self.tname


def assert_exists(path, isfile=False, isdir=False, msg=None):
    if os.path.exists(path):
        if isdir and not os.path.isdir(path):
            if msg is None:
                print path + ' must be a directory'
            else:
                print msg + ': ' + path
            exit(1)
        if isfile and not os.path.isfile(path):
            if msg is None:
                print path + ' must be a file'
            else:
                print msg + ': ' + path
            exit(1)
    else:
        if msg is None:
            print path + ' doesn\'t exist'
        else:
            print msg + ': ' + path
        exit(1)


def get_experiment_trajectories(expr_dir, params_file):
    traj_data = {}
    traj_files = [d for d in os.listdir(expr_dir) if os.path.splitext(d)[1] == ".pickle"]

    for traj in traj_files:
        path = os.path.join(expr_dir, traj)
        tname = os.path.splitext(traj)[0]
        traj_data[tname] = Trial(tname, path, params_file)

    return traj_data


def cleanup(proc):
    try:
        ret = proc.wait()  # timeout=600)
    except subprocess.TimeoutExpired:
        print "Timed out"
        proc.kill()
        ret = proc.wait(timeout=3)
    print "--- exit: {}".format(ret)


def run_one(trial_dir, n, trial):
    simtimeout = 20.0
    trial_dir = os.path.join(trial_dir, str(n))
    try:
        os.makedirs(trial_dir)
    except OSError:
        print 'Error making trial dir'
        return

    def tfile(x):
        return os.path.join(trial_dir, x)

    print "== Trial: {}, N: {}".format(trial_dir, n)

    # start rosbag
    proc = subprocess.Popen(['python', 'src/run_once.py',
                             trial.descpath,
                             tfile("out.pickle"),
                             trial.params_file,
                             "--sim_timeout", str(simtimeout)])
    time.sleep(2.0)
    cleanup(proc)


def ensurethreads(threads, max_threads):
    while len(threads) > max_threads:
        for i in range(len(threads)):
            if not threads[i].is_alive():
                threads[i].join()
                del threads[i]
                return


def run_experiments(nthreads, expr_dir, expr_name, params_file, out_dir):
    trajs = get_experiment_trajectories(expr_dir, params_file)
    ntrial = 10

    total = ntrial * len(trajs.items())
    j = 0
    shutil.copytree(expr_dir, out_dir)
    shutil.copyfile(params_file, out_dir + "/params.yaml")

    threads = []
    tlock = threading.Lock()
    for trajname, traj in trajs.items():
        exp_dir = os.path.join(out_dir, trajname)
        os.makedirs(exp_dir)

        trial_dir = os.path.join(exp_dir, trajname)
        os.makedirs(trial_dir)

        trial_dir = os.path.join(exp_dir, trajname)
        for i in range(ntrial):
            with tlock:
                ensurethreads(threads, nthreads)
                t = threading.Thread(target=run_one, args=(trial_dir, i, traj))
                t.start()
                threads.append(t)
                time.sleep(0.5)
                j += 1
                print "Trial %d of %d" % (j, total)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs a test suite for mushr_rhc')
    parser.add_argument('expr_dir',
                        help='Directory to read and run experiments from')
    parser.add_argument('out_dir',
                        help='Directory to output expermiment data to')
    parser.add_argument('params_file',
                        help='Parameter file for the RHC')
    parser.add_argument('--expr-name',
                        nargs='+',
                        default='all',
                        help='Experiment to run')
    parser.add_argument('--real',
                        action='store_true',
                        help='Run experiments in real')
    parser.add_argument('--prompt-comments',
                        action='store_true',
                        help='Prompt for comment on the run after every run')

    args, _ = parser.parse_known_args()

    # Ignore ros specific parameters
    args.expr_name = list(filter(lambda x: '__' not in x, args.expr_name))

    assert_exists(args.expr_dir, isdir=True)
    if os.path.exists(args.out_dir):
        t = str(dt.datetime.now()).replace(' ', '-')
        print args.out_dir, args.out_dir + '-' + t
        os.rename(args.out_dir, args.out_dir + '-' + t)

    run_experiments(1, args.expr_dir, args.expr_name, args.params_file, args.out_dir)
