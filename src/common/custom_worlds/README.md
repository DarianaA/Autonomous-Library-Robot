## Instructions

Models directory contains the definition of objects and features that can be added in the simulated world, such as books, cans, people.


To launch library world:

    roslaunch custom_worlds wrs.launch
    **This command will open Gazebo, Rviz and spawn TIAGo at the centre of the library world.


To launch custom_worlds in mapping mode:

    roslaunch custom_worlds tiago_mapping.launch world:="<world_name>"


To launch custom_worlds in navigation mode:

    roslaunch custom_worlds tiago_navigation.launch world:="<world_name>"


The script **/scripts/spawn_objects** will load the world with objects on the table and on the floor. To execute it:
        
    rosrun custom_worlds spawn_objects


To make the robot move to the table run:

	rosrun custom_worlds move.py


## Directories

- Worlds can be found in and added to the **/worlds** directory.
- 3D models can be found in and added to the **/models** directory.
- Quick-launch files for navigation can be found in the **/launch** directory.
