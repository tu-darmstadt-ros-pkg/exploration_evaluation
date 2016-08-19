#!/bin/bash
# Bash script that launches exploration_evaluation either in an infinite
# loop (if no optional parameter passed) or $1 times.
#
# Parameter:
# $1: Number of times script gets called.
# $2: Path to script that copies geotiff maps (and its parameters).
COUNTER=1

# run $1 times
if [ ! -z "$1" ]; then
    
    while [ $COUNTER -le $1 ]
    do
        roscore&
        sleep 10
        roslaunch exploration_evaluation launch_eval_complete.launch
        # Saving geotiff if $2 exists.
        if [ ! -z "$2" ]; then
            $2 $COUNTER
        fi
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
        roslaunch exploration_evaluation launch_eval_complete.launch
        echo Ran $COUNTER experiments
        ((COUNTER+=1))
        sleep 10
        pkill roscore
        pkill gazebo
        pkill apport
        sleep 10
    done
fi
