# exploration_evaluation
Scripts for evaluation of exploration behaviors.

# Starting the Script with Default Paramters
Use the _run\_looped.sh_ script (that can be found in the scripts directory). And call it by:
```
./scripts/run_looped.sh
```
This will start exploration once, with simulation time 600, the default behavior is "Search Victims" and the default arena maze\_many\_victims.

# Starting the Script Parameterized
The script for running exploration\_evaluation has a lot of parameters
 - NUM: Number of trials that will be executed
 - TIME: Time each trial has in simulation
 - BEHAVIOR: FlexBe behavior that will be called in simulation.
 - ARENA: Arena (from hector\_nist\_arenas) that will be used.
 - (optional: SCRIPT: Path to script that will be called after each trial.)

Use the _run\_looped.sh_ script (that can be found in the scripts directory). And call it by:
```
./scripts/run_looped.sh <NUM> <TIME> <BEHAVIOR> <ARENA> (<SCRIPT>)
```

Here is an example:
```
./scripts/run_looped.sh 5 600 "Search Victims" maze_many_victims
```
This will start the exploration\_evaluation 5 times in a row. Each trial has a simulationtime of 600 seconds. The behavior "Search Vicitms" will be used as behavior and everything is simulated in the maze\_many\_victims arena.


