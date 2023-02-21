import subprocess
import time

# Start roscore
subprocess.Popen(['roscore'])

# Wait for a few seconds to let roscore start
time.sleep(5)

# Launch Gazebo world
subprocess.Popen(['export', 'TURTLEBOT3_MODEL=waffle_pi'])
subprocess.Popen(['roslaunch', 'turtlebot3_gazebo', 'turtlebot3_world.launch'])
time.sleep(30)

# Launch SLAM
subprocess.Popen(['export', 'TURTLEBOT3_MODEL=waffle_pi'])
subprocess.Popen(['roslaunch', 'turtlebot3_slam', 'turtlebot3_slam.launch', 'slam_methods:=gmapping'])
time.sleep(30)

# Launch Gazebo simulation
subprocess.Popen(['roslaunch', 'turtlebot3_gazebo', 'turtlebot3_simulation.launch'])

# Wait for 5 minutes
time.sleep(300)

# Save map
subprocess.Popen(['rosrun', 'map_server', 'map_saver', '-f', '~/map_test'])
time.sleep(5)

# Kill all processes
subprocess.Popen(['killall', 'roslaunch'])
subprocess.Popen(['killall', 'gzserver'])
subprocess.Popen(['killall', 'gzclient'])

# Launch Gazebo world with saved map
subprocess.Popen(['export', 'TURTLEBOT3_MODEL=waffle_pi'])
subprocess.Popen(['roslaunch', 'turtlebot3_gazebo', 'turtlebot3_world.launch'])
time.sleep(30)

# Launch navigation with saved map
subprocess.Popen(['export', 'TURTLEBOT3_MODEL=waffle_pi'])
subprocess.Popen(['roslaunch', 'turtlebot3_navigation', 'turtlebot3_navigation.launch', 'map_file:=$HOME/map_test.yaml'])
