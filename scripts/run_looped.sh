#!/bin/bash
# Bash script that launches exploration_evaluation either in an infinite
# loop (if no optional parameter passed) or $1 times.
COUNTER=1

# run $1 times
if [ ! -z "$1" ]; then
    
    while [ $COUNTER -le $1 ]
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

# run infinite
else 
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
fi
