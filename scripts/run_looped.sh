#!/bin/bash


COUNTER=0

while :
do
        optirun roslaunch exploration_evaluation launch_eval_complete.launch
        echo Ran $COUNTER experiments
        sleep 10
        pkill gazebo
        pkill apport
        sleep 10
done