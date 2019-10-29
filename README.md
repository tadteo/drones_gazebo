# DRONES_GAZEBO
An autonomous system systems of drones collision avoidance implementations
## Usage
Install gazebo export path of model/world/plugins to .bashrc file
generetare world file wiht
```
python generate.py -n <number_of_drones> -a <algorithm>

<algorithm>: ORCA / BAPF / EAPF
```
Run the simulation on gazebo with
```
gazebo drones_<number of drones>.world
```