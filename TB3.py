#!/usr/bin/env python

import rospy
import os
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, Pose
from tf.transformations import euler_from_quaternion
from tf import TransformListener
from std_srvs.srv import Empty
from map_server.srv import LoadMap, LoadMapRequest
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class Turtlebot3:

    def __init__(self):
        # Create a publisher for robot velocity commands
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Initialize the TransformListener
        self.listener = TransformListener()

    def scan_callback(self, msg):
        # Calculate the average distance to obstacles
        distances = []
        for i in range(90, 270):
            if msg.ranges[i] < msg.range_max:
                distances.append(msg.ranges[i])
        if distances:
            avg_distance = sum(distances) / len(distances)
        else:
            avg_distance = 0.0

        # Stop the robot if it gets too close to an obstacle
        if avg_distance < 0.5:
            self.set_velocity(0.0, 0.0)

    def set_velocity(self, linear, angular):
        # Create a Twist message and publish it
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.velocity_pub.publish(twist)

    def build_map(self):
        # Start the mapping node
        os.system("roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping")

        # Wait for mapping to complete
        rospy.sleep(60)

        # Save the map
        os.system("rosrun map_server map_saver -f ~/map")

        # Stop the mapping node
        rospy.wait_for_service('/gazebo/reset_simulation')
        reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        reset_sim()
        rospy.sleep(5)

    def load_map(self, filename):
        # Load a map from file
        load_map = rospy.ServiceProxy('/map_server/load_map', LoadMap)
        req = LoadMapRequest()
        req.filename = filename
        load_map(req)

    def set_navigation_goal(self, x, y, theta):
        # Initialize the move_base client
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        # Create a Pose object with the goal coordinates
        goal_pose = Pose()
        goal_pose.position.x = x
        goal_pose.position.y = y
        goal_pose.position.z = 0.0
        quat = euler_from_quaternion([0, 0, theta, 1])
        goal_pose.orientation.x = quat[0]
        goal_pose.orientation.y = quat[1]
        goal_pose.orientation.z = quat[2]
        goal_pose.orientation.w = quat[3]

        # Create a MoveBaseGoal with the goal pose
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = goal_pose

        # Send the goal and wait for it to complete
        client.send_goal(goal)
        client.wait_for_result()

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('turtlebot3_node')

    # Create a Turtlebot3 object
    turtlebot = Turtlebot3()

    # Start the simulation
    os.system("roslaunch turtlebot3_gazebo turtlebot3_world.launch")

    # Wait for the simulation to start up
    rospy.sleep(5)

    # Build the map
    turtlebot.build_map()

    # Load the map
    turtlebot.load_map('~/map.yaml')

    # Set a navigation goal
    turtlebot.set_navigation_goal(2.0, 2.0, 0.0)

    # Spin until the node is shut down
    rospy.spin()
