# Pure pursuit path tracker
This code consists of implementation of pure pursuit path tracking algorithm.
It tracks a given reference path given to it. Currently it reads waypoints from a file and publishes ackerman steering
commands. Topic names are defined in the config file.
 
## Usage 
__1. Run the F1/10 simulator.__
```sh
roslaunch pure_pursuit_tracker pure_pursuit_simulator.launch
```
__2. Run pure pursuit controller node.__
```sh
roslaunch pure_pursuit_tracker run_sim_pure_pursuit.launch
```
You can change the parameters of the pure pursuit tracker in the config file pure_pursuit_sim.yaml located inside config folder.

