#!/usr/bin/env python

import rospy
import subprocess

def main():
    # Launch the turtlebot3 waffle pi in gazebo
    subprocess.Popen(['roslaunch', 'turtlebot3_gazebo', 'turtlebot3_world.launch'])

    # Wait for the simulation to start up
    rospy.sleep(5)

    # Launch SLAM
    subprocess.Popen(['roslaunch', 'turtlebot3_slam', 'turtlebot3_slam.launch'])

    # Wait for SLAM to start up
    rospy.sleep(5)

    # Drive the turtlebot3 waffle pi around the area to map out the environment
    subprocess.Popen(['roslaunch', 'turtlebot3_teleop', 'turtlebot3_teleop_key.launch'])

    # Wait for the mapping to be completed
    rospy.sleep(60)

    # Save the map
    subprocess.Popen(['rosrun', 'map_server', 'map_saver', '-f', '/path/to/map_file_name'])

    # Wait for the map to be saved
    rospy.sleep(5)

    # Launch the turtlebot3 waffle pi in gazebo with the map
    subprocess.Popen(['roslaunch', 'turtlebot3_gazebo', 'turtlebot3_world.launch', 'map_file:=/path/to/map_file_name.yaml'])

    # Wait for the simulation to start up
    rospy.sleep(5)

    # Launch RViz
    subprocess.Popen(['roslaunch', 'turtlebot3_navigation', 'turtlebot3_navigation.launch', 'map_file:=/path/to/map_file_name.yaml'])

    # Wait for RViz to start up
    rospy.sleep(5)

    # Set the 2D Pose Estimate in RViz to indicate the turtlebot3 waffle pi's starting location on the map
    subprocess.Popen(['rostopic', 'pub', '/initialpose', 'geometry_msgs/PoseWithCovarianceStamped', '-1', '0', '0', '0', '0', '0', '1'])

    # Wait for the 2D Pose Estimate to be set
    rospy.sleep(5)

    # Set a navigation goal in RViz
    subprocess.Popen(['rostopic', 'pub', '/move_base_simple/goal', 'geometry_msgs/PoseStamped', '-1', '{ header: { stamp: now, frame_id: "map" }, pose: { position: { x: 1.0, y: 1.0, z: 0.0 }, orientation: { w: 1.0 } } }'])

    # Wait for the navigation goal to be set
    rospy.sleep(5)

if __name__ == '__main__':
    main()