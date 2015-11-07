#!/bin/bash


COUNTER=1

while :
do
        roscore&
        sleep 10
        optirun roslaunch exploration_evaluation launch_eval_complete.launch
        echo Ran $COUNTER experiments
        ((COUNTER+=1))
        sleep 10
        pkill roscore
        pkill gazebo
        pkill apport
        sleep 10
done