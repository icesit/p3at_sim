# navigation of pioneer3at
## statement
* just a movebase
* can work with cartographer, put cartographerfile to the right place
## run cartographer
* roslaunch cartographer_ros sim_backpack_2d.launch -- (build map)
* rosservice call /write_state ${HOME}/catkin_ws/src/amr-ros-config/savefile/simmap.pbstream -- (save map)
* roslaunch cartographer_ros sim_backpack_2d_localization.launch load_state_filename:=/home/xuewuyang/catkin_ws/src/amr-ros-config/savefile/simmap.pbstream -- (load map)
## requirement
* sudo apt install ros-kinetic-navigation
* I cannot remember more...
