#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import OrientationConstraint, Constraints
from controller import Controller

import numpy as np
from numpy import linalg
import sys
from sensor_msgs.msg import JointState
from intera_interface import Limb

# from intera_interface import gripper as robot_gripper


def ik_solver(endEffectorPose: PoseStamped):
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    # rospy.init_node('service_query')

    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    # while not rospy.is_shutdown():

    # Construct the request
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"

    # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
    link = "right_gripper_tip"

    request.ik_request.ik_link_name = link
    # request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"
    request.ik_request.pose_stamped = endEffectorPose

    try:
        # Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
        # Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        # Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        # Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
        # controller = Controller(Kp, Ki, Kd, Kw, Limb("right"))
        print("reached here 2")
        # Send the request to the service
        response = compute_ik(request)

        # Print the response HERE
        # return response
        group = MoveGroupCommander("right_arm")

        # Setting pfrom sensor_msgs.msg import JointStateosition and orientation target
        # group.set_pose_target(request.ik_request.pose_stamped)

        constraints = Constraints()
        orien_const = OrientationConstraint()
        orien_const.link_name = "right_hand"
        orien_const.header.frame_id = "base"
        orien_const.orientation.y = 1.0
        constraint_tolerance = 0.3
        orien_const.absolute_x_axis_tolerance = constraint_tolerance
        orien_const.absolute_y_axis_tolerance = constraint_tolerance
        orien_const.absolute_z_axis_tolerance = constraint_tolerance
        orien_const.weight = 1.0

        constraints.orientation_constraints = [orien_const]
        # group.set_path_constraints(constraints)
        # TRY THIS
        # Setting just the position without specifying the orientation
        ###group.set_position_target([0.5, 0.5, 0.0])
        # print(type(response))
        # print(dir(response.solution))
        # print(response.solution)



        # angles = np.asarray(response.solution.joint_state.position[2:9], dtype='float64')

        # print(response.solution.joint_state.name)


        joint_states = JointState()
        joint_states.header.frame_id = "base"
        joint_states.name = response.solution.joint_state.name[0:1] + response.solution.joint_state.name[2:8]
        joint_states.position = response.solution.joint_state.position[0:1] + response.solution.joint_state.position[2:8]

        # Plan IK
        planned_path = group.plan(joint_states)
        # while len(planned_path) != 4:
        #     planned_path = group.plan(joint_states)
        # planned_path = None
        # planned_time = np.inf
        # for _ in range(100):
        #     plan = group.plan(request.ik_request.pose_stamped.pose)
        #     if plan[2] < planned_time:
        #         planned_time = plan[2]
        #         planned_path = plan
        # print(planned_time)
        # user_input = input(
        #     "Enter 'y' if the trajectory looks safe on RVIZ")

        # Execute IK if safe
        # if user_input == 'y':
        group.execute(planned_path[1])
            # print("trying to execute")
            # controller.execute_plan()
            # controller.execute(planned_path[1], log=False)
        #     # right_gripper.close()

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == '__main__':
    rospy.init_node("ik_solver")
    goal = PoseStamped()
    goal.header.frame_id = "base"
    goal.pose.orientation.x = 0
    goal.pose.orientation.y = 1
    goal.pose.orientation.z = 0
    goal.pose.orientation.w = 0
    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = [
        0.878, 0.209, -0.05]
    print(ik_solver(goal))

    # goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = [
    #     0.890, 0.209, -0.05]
    # ik_solver(goal)
