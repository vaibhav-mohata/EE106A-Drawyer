#!/usr/bin/env python
"""
Generates a path for Sawyer from a numpy array
Author: Heath Matthews, Vaibhav

### Run the following in terminal ###
code ~/.bashrc
source ~/.bashrc
catkin_make
source devel/setup.bash
rosrun intera_interface enable_robot.py -e
rosrun intera_interface joint_trajectory_action_server.py
roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true

"""
# import sys
# from intera_interface import Limb
import rospy
import numpy as np

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner
from ik_solver import ik_solver

# try:
#     from controller import Controller
# except ImportError:
#     pass
    
def main():
    coords = np.array([(.6, 0., .3),(.6, 0., .3),(.7, 0., .3)]) #this migth not be right
    draw_paths(coords)


def draw_paths(paths):

    # planner = PathPlanner("right_arm")

    # Add the table
    # size = np.array([0.4, 1.2, 0.1])
    # name = "table"
    # pose = PoseStamped()
    # pose.header.frame_id = "base"
    # pose.pose.position.x = 0.62
    # pose.pose.position.y = 0.0
    # pose.pose.position.z = -0.2
    # pose.pose.orientation.w = 1.0
    # pose.pose.orientation.x = 0.0
    # pose.pose.orientation.y = 0.0
    # pose.pose.orientation.z = 0.0
    # planner.add_box_obstacle(size, name, pose)

    # orien_const = OrientationConstraint()
    # orien_const.link_name = "right_gripper"
    # orien_const.header.frame_id = "base"
    # orien_const.orientation.y = -1.0
    # constraint_weight = 0.1
    # orien_const.absolute_x_axis_tolerance = constraint_weight
    # orien_const.absolute_y_axis_tolerance = constraint_weight
    # orien_const.absolute_z_axis_tolerance = constraint_weight
    # orien_const.weight = 1.0

    # Define controller type
    # Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
    # Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    # Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    # Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
    # # controller = Controller(Kp, Ki, Kd, Kw, Limb("right"))

    # For each 3D coord
    # rospy.init_node('moveit_node')
    while not rospy.is_shutdown():
        for path in paths:
            for coord in path:
                goal = PoseStamped()
                goal.header.frame_id = "base"
                goal.pose.position.x = coord[0]
                goal.pose.position.y = coord[1]
                goal.pose.position.z = coord[2]
                goal.pose.orientation.x = 0.0
                goal.pose.orientation.y = -1.0
                goal.pose.orientation.z = 0.0
                goal.pose.orientation.w = 0.0
                ik_solver(goal)
        # break
                # plan = planner.plan_to_pose(goal, [orien_const])
                # input("Press enter if it looks ok")
                # try:
                #     if not planner.execute_plan(plan[1]): 
                #         raise Exception("Execution failed")
                # except Exception as e:
                #     print(e)



if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
