#!/usr/bin/env python


import rospy, sys
import thread, copy
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from math import radians
from copy import deepcopy

class AttachMesh:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('AttachMesh')
        
        # Construct the initial scene object
        scene = PlanningSceneInterface()
        
        # Pause for the scene to get ready
        rospy.sleep(1)
                                
        # Initialize the MoveIt! commander for the right wam and fingers
        right_wam = moveit_commander.MoveGroupCommander('right_wam')

        right_wam_finger_1 = moveit_commander.MoveGroupCommander('right_wam_finger_1')
        right_wam_finger_2 = moveit_commander.MoveGroupCommander('right_wam_finger_2')
        right_wam_finger_3 = moveit_commander.MoveGroupCommander('right_wam_finger_3')
                
      
        
        # Get the name of the end-effector link
        end_effector_link = right_wam.get_end_effector_link()
        
        # Allow some leeway in position (meters) and orientation (radians)
        right_wam.set_goal_position_tolerance(0.01)
        right_wam.set_goal_orientation_tolerance(0.05)
       
        # Allow replanning to increase the odds of a solution
        right_wam.allow_replanning(True)
        
        # Allow 5 seconds per planning attempt
        right_wam.set_planning_time(5)
        
        # Remove leftover objects from a previous run
        scene.remove_attached_object(end_effector_link, 'spoon')
        
        right_wam.set_named_target('right_wam_start')
        right_wam.go()
        rospy.sleep(2)
        # Closing the hand first
        # Closing the hand
        right_wam_finger_1.set_named_target("right_wam_finger_1_grasp")
        right_wam_finger_2.set_named_target("right_wam_finger_2_grasp")
        right_wam_finger_3.set_named_target("right_wam_finger_3_grasp")
 
        right_wam_finger_1.execute(right_wam_finger_1.plan())
        rospy.sleep(5)   
        right_wam_finger_2.execute(right_wam_finger_2.plan())
        rospy.sleep(5)    
        right_wam_finger_3.execute(right_wam_finger_3.plan())  
        rospy.sleep(5)
        
        # Set the length, width and height of the object to attach
        #tool_size = [0.3, 0.02, 0.02]
        
        # Create a pose for the tool relative to the end-effector
        p = PoseStamped()
        p.header.frame_id = end_effector_link
        
        # Place the end of the object within the grasp of the gripper
        p.pose.position.x = 0.0
        p.pose.position.y = 0.0
        p.pose.position.z = -0.02
        
        # Align the object with the gripper (straight out)
        p.pose.orientation.x = -0.5
        p.pose.orientation.y = 0.5
        p.pose.orientation.z = -0.5
        p.pose.orientation.w = 0.5
        

        
        scene.attach_mesh(end_effector_link, 'spoon', p, '/home/yzheng/catkin_ws/src/manipulation_scenarios/ycb_object_models/models/stl/spoon.stl')
        

   
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    AttachMesh()
