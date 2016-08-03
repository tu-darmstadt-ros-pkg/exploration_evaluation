# exploration_evaluation
Scripts for evaluation of exploration behaviors.

# Starting the Scripts
The scripts can be started in two different ways:
 - Run once via launchfile 
 - Run looped via script

### Run once via Launchfile
You can use the _launch\_eval\_complete.launch_ launchfile:
```
roslaunch launch_eval_complete.launch
```
This will start exploration and evaluation on _exploration\_eval\_arena.world_. The launchfile is parameterized. If you want to use another world (from hector\_nist\_arena\_worlds), you can use:
```
roslaunch launch_eval_complete.launch world:=my_worldname
```

### Run looped via Script
If you want to run more than one trial of your exploration-evaluation, you can use the _run\_looped.sh_ script (that can be found in the scripts directory). You can call it by:
```
./scripts/run_looped.sh
```
This will start the script in an endless loop. If you only want to do a certain number of exploration-evaluation trials, you can call the script with the number of trials:
```
./scripts/run_looped.sh 5
```
This will start the exploration-evaluation script 5 times in a row.
