#!/bin/bash

TRAJ_DIR=kinks/
OUT_DIR=out/kinks_mujoco
PARAMS_FILE=~/catkin_ws/src/mushr_rhc/mushr_rhc_ros/launch/sim/params_block_sim.yaml

exec python src/run_expr.py $TRAJ_DIR $OUT_DIR $PARAMS_FILE
