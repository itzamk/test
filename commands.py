import os
import time

os.system('gnome-terminal -- roscore')
time.sleep(2)

os.system('gnome-terminal -- bash -c "export TURTLEBOT3_MODEL=waffle_pi; roslaunch turtlebot3_gazebo turtlebot3_world.launch"')
time.sleep(30)

os.system('gnome-terminal -- bash -c "export TURTLEBOT3_MODEL=waffle_pi; roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping"')
time.sleep(30)

os.system('gnome-terminal -- roslaunch turtlebot3_gazebo turtlebot3_simulation.launch')
time.sleep(600)

os.system('gnome-terminal -- rosrun map_server map_saver -f ~/map_test')
time.sleep(5)

os.system('pkill -f "roslaunch"')
time.sleep(5)

os.system('gnome-terminal -- bash -c "export TURTLEBOT3_MODEL=waffle_pi; roslaunch turtlebot3_gazebo turtlebot3_world.launch"')
time.sleep(30)

os.system('gnome-terminal -- bash -c "export TURTLEBOT3_MODEL=waffle_pi; roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map_test.yaml"')
