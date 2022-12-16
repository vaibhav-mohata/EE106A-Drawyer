#!/usr/bin/env python
"""
Path testing script for final project
"""
import sys
from intera_interface import Limb
import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner
from ik_solver import ik_solver
from forward_kinematics import forward_kinematics_from_angles

Z_VALUE_PEN_UP = 0.25
Z_VALUE_PEN_DOWN = 0.00

def convertDFSToPlans(dfsTreePoints):

    planner = PathPlanner("right_arm")

    orien_const = OrientationConstraint()
    orien_const.link_name = "right_hand";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0;
    constraint_weight = 0.2
    orien_const.absolute_x_axis_tolerance = constraint_weight;
    orien_const.absolute_y_axis_tolerance = constraint_weight;
    orien_const.absolute_z_axis_tolerance = constraint_weight;
    orien_const.weight = 1.0;

    while not rospy.is_shutdown():
        for path in dfsTreePoints:
            while not rospy.is_shutdown():
                goal = PoseStamped()
                goal.header.frame_id = "base"
                goal.pose.orientation.x = 0
                goal.pose.orientation.y = 1
                goal.pose.orientation.z = 0
                goal.pose.orientation.w = 0

                try:
                    goal.pose.position.x, goal.pose.position.y = path[0]
                    goal.pose.position.z = Z_VALUE_PEN_UP
                    plan = planner.plan_to_pose(goal, [orien_const])
                    if not planner.execute_plan(plan[1]): 
                        raise Exception("Execution failed")
                    for points in path:
                        goal.pose.position.x, goal.pose.position.y = points
                        goal.pose.position.z = Z_VALUE_PEN_DOWN
                        plan = planner.plan_to_pose(goal, [orien_const])
                        if not planner.execute_plan(plan[1]): 
                            raise Exception("Execution failed")
                    goal.pose.position.x, goal.pose.position.y = path[-1]
                    goal.pose.position.z = Z_VALUE_PEN_UP
                    plan = planner.plan_to_pose(goal, [orien_const])
                    if not planner.execute_plan(plan[1]): 
                        raise Exception("Execution failed for path ", path)
                except Exception as e:
                    print(e)
                    traceback.print_exc()
                else:
                    break




def main():
    """
    Main Script
    """

    # Make sure that you've looked at and understand path_planner.py before starting

    planner = PathPlanner("right_arm")


    Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
    # robot_controller = Controller(Kp, Ki, Kd, Kw, Limb('right'))




    # # 
    # # Add the obstacle to the planning scene here
    # #

    # #Create a path constraint for the arm
    # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_hand";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = 1.0;
    constraint_tolerance = 0.3
    orien_const.absolute_x_axis_tolerance = constraint_tolerance;
    orien_const.absolute_y_axis_tolerance = constraint_tolerance;
    orien_const.absolute_z_axis_tolerance = constraint_tolerance;
    orien_const.weight = 1.0;

    table_pos = PoseStamped()
    table_pos.header.frame_id = "base"
    table_pos.pose.orientation.x = 1.00
    table_pos.pose.orientation.y = 0.00
    table_pos.pose.orientation.z = 0.00
    table_pos.pose.orientation.w = 0.00

    table_pos.pose.position.x = 0.70
    table_pos.pose.position.y = 0.00
    table_pos.pose.position.z = -0.20

    # box_obstacle_pose.pose.position.x = 0.50
    # box_obstacle_pose.pose.position.y = 0.00
    # box_obstacle_pose.pose.position.z = 0.00

    planner.add_box_obstacle(np.array([0.40, 1.20, 0.10]), 'Table', table_pos)

    box_obstable_pos = PoseStamped()
    box_obstable_pos.header.frame_id = "base"
    box_obstable_pos.pose.orientation.x = 1.00
    box_obstable_pos.pose.orientation.y = 0.00
    box_obstable_pos.pose.orientation.z = 0.00
    box_obstable_pos.pose.orientation.w = 0.00

    box_obstable_pos.pose.position.x = 0.70
    box_obstable_pos.pose.position.y = 0.00
    box_obstable_pos.pose.position.z = 0.80

    planner.add_box_obstacle(np.array([0.40, 1.20, 0.10]), 'Upper limit', box_obstable_pos)

    box_obstable_pos_1 = PoseStamped()
    box_obstable_pos_1.header.frame_id = "base"
    box_obstable_pos_1.pose.orientation.x = 0.00
    box_obstable_pos_1.pose.orientation.y = 0.00
    box_obstable_pos_1.pose.orientation.z = 0.00
    box_obstable_pos_1.pose.orientation.w = 1.00

    box_obstable_pos_1.pose.position.x = 0.30
    box_obstable_pos_1.pose.position.y = 0.00
    box_obstable_pos_1.pose.position.z = 0.40

    box_obstable_pos_1.pose.position.x = -0.56
    box_obstable_pos_1.pose.position.y = 0.00
    box_obstable_pos_1.pose.position.z = 0.40

    planner.add_box_obstacle(np.array([0, 1, 1]), 'Back limit', box_obstable_pos_1)



    while not rospy.is_shutdown():

        while not rospy.is_shutdown():
            try:
                # x, y, z = 1.015, 0.160, 0.317 #### Position for top left
                x, y, z = [0.878, 0.209, 0.00]
                # x, y, z = 0.344, 0.206, 0.021
                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"
                goal_1.pose.orientation.x, goal_1.pose.orientation.y, goal_1.pose.orientation.z, goal_1.pose.orientation.w = [0, 1, 0, 0]
                #x, y, and z position
                goal_1.pose.position.x = x
                goal_1.pose.position.y = y
                goal_1.pose.position.z = z

                #Orientation as a quaternion
                # goal_1.pose.orientation.x = 0.0
                # goal_1.pose.orientation.y = -1.0
                # goal_1.pose.orientation.z = 0.0
                # goal_1.pose.orientation.w = 0.0

                # Might have to edit this . . . 
                # joint_angles = ik_solver(goal_1)

                plan = planner.plan_to_pose(goal_1, [orien_const])
                # print(plan)
                input("Press <Enter> to move the right arm to goal pose 1: ")
                if not planner.execute_plan(plan[1]): 
                    raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break

        while not rospy.is_shutdown():
            try:
                x, y, z = [0.878, 0.209, 0.1]
                # x, y, z = 0.344, 0.206, 0.021
                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"
                goal_1.pose.orientation.x, goal_1.pose.orientation.y, goal_1.pose.orientation.z, goal_1.pose.orientation.w = [0, 1, 0, 0]
                #x, y, and z position
                goal_1.pose.position.x = x
                goal_1.pose.position.y = y
                goal_1.pose.position.z = z

                plan = planner.plan_to_pose(goal_1, [orien_const])
                input("Press <Enter> to move the right arm to goal pose 2: ")
                if not planner.execute_plan(plan[1]):
                    raise Exception("Execution failed")
            except Exception as e:
                print(e)
            else:
                break

        while not rospy.is_shutdown():
            try:
                goal_3 = PoseStamped()
                goal_3.header.frame_id = "base"

                #x, y, and z position
                goal_3.pose.position.x = 0.6
                goal_3.pose.position.y = -0.1
                goal_3.pose.position.z = 0.1

                #Orientation as a quaternion
                goal_3.pose.orientation.x = 0.0
                goal_3.pose.orientation.y = -1.0
                goal_3.pose.orientation.z = 0.0
                goal_3.pose.orientation.w = 0.0

                plan = planner.plan_to_pose(goal_3, [])
                input("Press <Enter> to move the right arm to goal pose 3: ")
                if not robot_controller.execute_plan(plan[1]):
                    raise Exception("Execution failed")
            except Exception as e:
                print(e)
            else:
                break

if __name__ == '__main__':
    rospy.init_node('path_test')
    main()