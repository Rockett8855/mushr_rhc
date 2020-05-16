#!/bin/bash

TRAJ_DIR=turns_only/
OUT_DIR=out/turns_only_gpr
PARAMS_FILE=~/catkin_ws/src/mushr_rhc/mushr_rhc_ros/launch/sim/params_block_sim_gpr.yaml

exec python src/run_expr.py $TRAJ_DIR $OUT_DIR $PARAMS_FILE
