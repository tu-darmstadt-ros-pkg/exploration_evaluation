#!/bin/bash
# Bash script that launches exploration_evaluation either in an infinite
# loop (if no optional parameter passed) or $1 times.
#
# Parameter:
# $1: Number of times script gets called.
# $2: Length of sim time for each trial.
# $3: Behavior that will be started.
# $4: Arena that will be used.
# $5: Path to script that copies geotiff maps (and its parameters). See hector_jenkins repo.

COUNTER=1

if [ ! -z "$1" ]; then
  TRIALS=$1
else
  TRIALS=1
fi

if [ ! -z "$2" ]; then
  SIMTIME=$2
else
  SIMTIME="600"
fi

if [ ! -z "$3" ]; then
  BEHAVIOR=$3
else
  BEHAVIOR="Search Victims"
fi

if [ ! -z "$4" ]; then
  ARENA=$4
else
  ARENA="maze_many_victims"
fi

echo Simtime is $SIMTIME , behavior is $BEHAVIOR and arena is $ARENA

while [ $COUNTER -le $TRIALS ]
    do
        roscore&
        sleep 10
        roslaunch exploration_evaluation launch_eval_complete.launch mission_sim_time:=$SIMTIME mission_behavior:="$BEHAVIOR" arena:=$ARENA
        # Saving geotiff if $5 exists.
        if [ ! -z "$5" ]; then
            $5 $COUNTER
        fi
        echo Ran $COUNTER experiments
        ((COUNTER+=1))
        sleep 10
        pkill roscore
        pkill gazebo
        pkill apport
        sleep 10
    done

