#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function
import imp
#from urllib import response
from six.moves import input
import actionlib
import franka_gripper.msg

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import sensor_msgs.msg
from shape_msgs.msg import Plane
import geometry_msgs.msg
import trajectory_msgs.msg
from std_msgs.msg import Float32, Int16
try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

import math
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from panda_msgs.srv import PandaSimpleService, PandaSimpleServiceResponse, PandaPickUp, PandaPickUpResponse, PandaManyPoses, PandaManyPosesResponse
from panda_msgs.msg import FloatList
from collision_scene_example import CollisionSceneExample 
from scipy.spatial.transform import Rotation
from franka_gripper_python import grasp_client_close, grasp_client_open
## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class UnityPythonConnector(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(UnityPythonConnector, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("unity_python", anonymous=True)
        joint_state_publisher_Unity = rospy.Publisher("/joint_state_unity", FloatList , queue_size = 10)
        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()
        # robot = moveit_msgs.msg.RobotState()
        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()
        tau = 2 * math.pi
        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name, wait_for_servers=15.0)
        hand_group = "panda_hand"
        move_group_hand = moveit_commander.MoveGroupCommander(hand_group, wait_for_servers=15.0)
        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        
        self.plan_publisher = rospy.Publisher(
            "plan_publisher",
            moveit_msgs.msg.RobotTrajectory,
            queue_size=20,
        )
        self.pose_n = 0

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # # We can get a list of all the groups in the robot:
        # group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        # print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL
        self.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5',
                    'panda_joint6','panda_joint7']
        self.hand_joint_names = ['panda_finger_joint1', 'panda_finger_joint2']
        move_group.set_end_effector_link("panda_hand")
        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.move_group_hand = move_group_hand
        self.pose_n_total = 5
        # self.group_names = group_names
        # self.add_plane_to_the_scene()
        print("============ publish current joint state")
        print("franka joints {}".format(move_group.get_current_joint_values()))
        print("hand joints {}".format(move_group_hand.get_current_joint_values()))

        joint_state_publisher_Unity.publish(move_group.get_current_joint_values())
        print("done")
        # self.robot = moveit_msgs.msg.RobotState() # may need to uncomment
        # hand rotation 
        # self.zero_rot = Rotation.from_euler('xyz', [0, 180, 0], degrees=True).as_quat()

        # self.add_plane_to_the_scene()
        # self.add_box() # adding table to the plannig space
        self.stackofplans = [] 
        

    def generate_JointMsg(self, joints):
        # returns the message with joint states in the form RobotState accepts it 
        # print("{} {}".format(len(joints), len(self.joint_names)))
        
        newjoint_state = sensor_msgs.msg.JointState()
        for i in range(len(self.joint_names)):
            newjoint_state.name.append(self.joint_names[i])
            newjoint_state.position.append(joints[i])
        return newjoint_state

    def plan_trajectory(self, init_angles, target_pose, verbose = False):
        # plan the traj given angles and target pose 
        robot = moveit_msgs.msg.RobotState()
        robot.joint_state = self.generate_JointMsg(init_angles)
        # self.robot.joint_state = self.generate_JointMsg(init_angles)
        self.move_group.set_start_state(robot)     
        self.move_group.set_pose_target(target_pose)
        if verbose:
            plan = self.move_group.plan()[1]
            print(plan)
            return plan
        return self.move_group.plan()[1]
    
    def go_to_pose_goal(self, request):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        # print("service was called")
        # print("pick up cube try")
        # self.move_group.pick("box")
        # print("success?")
        move_group = self.move_group
        # to set up initial joint state we need to define the message as a jointState type msg
        newjoint_state = sensor_msgs.msg.JointState()
        for i in range(len(self.joint_names)):
            newjoint_state.name.append(self.joint_names[i])
            newjoint_state.position.append(request.current_joints[i])
        # setting up initial joint state 
        robot = moveit_msgs.msg.RobotState()
        robot.joint_state = newjoint_state
        print(newjoint_state)
        move_group.set_start_state(robot)
    
        resp = PandaSimpleServiceResponse()
        move_group.set_pose_target(request.targetpose)
        plan = move_group.plan()
        resp.trajectories.append(plan[1])
        print(resp)
        return resp

    def pickup_plan(self, request):
        # request contain the message of pick up pose and place pose 
        # we need to return path to pre pick up pose
        # plan to pick up 
        # plan pack to pre pick up pose 
        # plan to placing position
        print(request)
        # print(request.pick_pose)
        self.plans = []
        resp = PandaPickUpResponse()
        
        # to set up initial joint state we need to define the message as a jointState type msjoint_trajectoryg       
        # setting up initial joint state 
        # 1 go to pre-pick up pose 
        # pick_pose_rotation = copy.deepcopy(request.pick_pose.orientation)
        request.pick_pose.position.z = 0.4
        # request.pick_pose.orientation = self.zero_rot
        plan = self.plan_trajectory(request.current_joints, request.pick_pose)
        # self.plan_publisher.publish(plan)
        self.plans.append(plan)        
        previous_ending_joint_angles = plan.joint_trajectory.points[-1].positions
        # 2 pick up pose 
        pick_pose = copy.deepcopy(request.pick_pose)
        pick_pose.position.z = 0.3
        plan = self.plan_trajectory(previous_ending_joint_angles, pick_pose)
        # print(plan)
        self.plans.append(plan)   
        previous_ending_joint_angles = plan.joint_trajectory.points[-1].positions
        # pint 2.5 just pick up
        # pick_pose.orientation = pick_pose_rotation # rotate to the state of the 
        plan = self.plan_trajectory(previous_ending_joint_angles, pick_pose)
        self.plans.append(plan)
        # 3 come back to pre-pick up pose 
        plan = self.plan_trajectory(previous_ending_joint_angles, request.pick_pose)
        self.plans.append(plan)
        previous_ending_joint_angles = plan.joint_trajectory.points[-1].positions
        # 4 come back to pre-pick up pose 
        request.place_pose.position.z = 0.4
        plan = self.plan_trajectory(previous_ending_joint_angles, request.place_pose)
        self.plans.append(plan)
        resp.trajectories = self.plans
        print("I renturned trajectory")
        # for p in self.plans:
        #     self.plan_publisher.publish(p)
        # try carthesian path
        # wpose = self.move_group.get_current_pose().pose
        # interpose = copy.deepcopy(request.pick_pose)
        # interpose.position.x = 0.26
        # interpose.position.z = -0.363
        # points = [wpose, request.pick_pose, pick_pose, request.pick_pose, interpose, request.place_pose]
        # plan = self.plan_cartesian_path(points)
        # plans = [plan]
        # resp.trajectories = plans
        # self.execute_plans(self.plans)
        return resp

    def pick_no_place(self, request):
        """
        request to move around without picking or placing the objects
        """
        response = PandaManyPosesResponse()

        # group_name = "arm"
        # move_group = moveit_commander.MoveGroupCommander(group_name)

        # current_robot_joint_configuration = request.current_joints
        robot_poses = []
        # print(f"this is PICK AND PLACE \n request : {request} \n current state: {request.current_joints}")

        # Pre grasp - position gripper directly above target object
        for i in range(len(request.poses)):
            if (i == 0):
                robot_poses.append(self.plan_trajectory(request.current_joints, request.poses[0]))

                # robot_poses.append(plan_trajectory(move_group, req.poses[0], current_robot_joint_configuration))
            else:
                robot_poses.append(self.plan_trajectory(robot_poses[i-1].joint_trajectory.points[-1].positions, request.poses[i]))
                # robot_poses.append(plan_trajectory(move_group, req.poses[i], robot_poses[i-1].joint_trajectory.points[-1].positions))
        
            if not robot_poses[i].joint_trajectory.points:
                return response
        response.trajectories = robot_poses
        # If trajectory planning worked for all pick and place stages, add plan to response
        print("im good boy i returned responsed")
        return response
    
    def execute_plans(self, plan):
        # executes plans on the real world robot (or rviz robot)
        # for plan in plans:
        self.pose_n += 1
        print(self.pose_n)        
        self.move_group.execute(plan, wait=True)
        # if ((self.pose_n == 2 and self.pose_n_total == 5) or (self.pose_n == 2 and self.pose_n_total > 5)):
        if (self.pose_n == 2):
            grasp_client_close()
        if (self.pose_n == self.pose_n_total):
            grasp_client_open()        
        # hand_movement
        # open_hand_plan = self.open_hand(self.move_group_hand.get_current_joint_values())
        # open_hand_state = open_hand_plan.joint_trajectory.points[-1].positions
        # close_hand_plan = self.close_hand(open_hand_state)
        # plan_hand = [open_hand_plan, close_hand_plan]
        # self.move_group_hand.execute(plan_hand)

    def assign_nposes(self,response):
        self.pose_n_total = response.data

def main():
    try:
        
        tutorial = UnityPythonConnector()
        # tutorial.add_box()
        coll = CollisionSceneExample()
        # coll.add_table()
        # add wall behind the robot 
        pose = [-0.6, 0, 0, 0, 0.707, 0, 0.707]
        dimensions = [1.2, 1.2, 0.0001]
        coll.add_table(pose, dimensions, "wall")
        # # add box to pick up
        # pose = [0.5, 0.2, 0.1, 0, 0., 0, 1]
        # dimensions = [0.03, 0.03, 0.03]
        rot = Rotation.from_euler('xyz', [0, 0, 90], degrees=True)
        rot = rot.as_quat()
        pose = [0.38, 0, 0.03, rot[0], rot[1], rot[2], rot[3]] # in robot frame so table is Y 0
        dimensions = [1.5, 0.28, 0.3] # this are dims for RViz not unity so X,Y,Z (Unity Z, X, Y) - possible to flip it 
        coll.add_table(pose, dimensions, "carton_box")
        print("Ready to plan")
        s = rospy.Service('moveit_many', PandaSimpleService, tutorial.go_to_pose_goal) # go_to_pose_goal
        s2 = rospy.Service('panda_move', PandaPickUp, tutorial.pickup_plan)
        s3 = rospy.Service('waypoints_service', PandaManyPoses, tutorial.pick_no_place)
        # sub1 = rospy.Subscriber('plan_publisher', moveit_msgs.msg.RobotTrajectory, tutorial.execute_plans)
        sub2 = rospy.Subscriber('realrobot_publisher', moveit_msgs.msg.RobotTrajectory, tutorial.execute_plans)
        sub3 = rospy.Subscriber('total_poses_n', Int16, tutorial.assign_nposes)
        # s4 = rospy.Service('waypoints_service', moveit_msgs.msg.RobotTrajectory, tutorial.execute_plans)
        rospy.spin()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
