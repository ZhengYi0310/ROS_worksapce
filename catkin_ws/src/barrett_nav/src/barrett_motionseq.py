#!/usr/bin/env python

import rospy, sys
import math
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

REFERENCE_FRAME = 'rack1'

class MotionSequence:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('MotionSequence')
        
   
        # Use the planning scene object to add or remove objects
        scene = PlanningSceneInterface()
        
        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene)

        # Create a publisher for displaying left wam finger poses
        self.left_wam_finger_1_pub = rospy.Publisher('left_wam_finger_1', PoseStamped)
        self.left_wam_finger_2_pub = rospy.Publisher('left_wam_finger_2', PoseStamped)
        self.left_wam_finger_3_pub = rospy.Publisher('left_wam_finger_3', PoseStamped)

     

        # Create a dictionary to hold object colors
        self.colors = dict()

        # Define move group comander for each moveit group
        left_wam = moveit_commander.MoveGroupCommander('left_wam')
        right_wam = moveit_commander.MoveGroupCommander('right_wam')
        left_wam_finger_1 = moveit_commander.MoveGroupCommander('left_wam_finger_1')
        left_wam_finger_2 = moveit_commander.MoveGroupCommander('left_wam_finger_2')
        left_wam_finger_3 = moveit_commander.MoveGroupCommander('left_wam_finger_3')
        right_wam_finger_1 = moveit_commander.MoveGroupCommander('right_wam_finger_1')
        right_wam_finger_2 = moveit_commander.MoveGroupCommander('right_wam_finger_2')
        right_wam_finger_3 = moveit_commander.MoveGroupCommander('right_wam_finger_3')

        left_wam.set_planner_id("PRMstarkConfigDefault")
        right_wam.set_planner_id("PRMstarkConfigDefault")
        #left_wam_finger_1.set_planner_id("RRTstarkConfigDefault")
        #left_wam_finger_2.set_planner_id("RRTstarkConfigDefault")
        #left_wam_finger_3.set_planner_id("RRTstarkConfigDefault")
        #right_wam_finger_1.set_planner_id("RRTstarkConfigDefault")
        #right_wam_finger_2.set_planner_id("RRTstarkConfigDefault")
        #right_wam_finger_3.set_planner_id("RRTstarkConfigDefault")


        # Get the name of the end-effector link
        left_end_effector_link = left_wam.get_end_effector_link()
        right_end_effector_link = right_wam.get_end_effector_link()

        # Display the name of the end_effector link
        rospy.loginfo("The end effector link of left wam is: " + str(left_end_effector_link))
        rospy.loginfo("The end effector link of right wam is: " + str(right_end_effector_link))

        # Allow some leeway in position (meters) and orientation (radians)
        right_wam.set_goal_position_tolerance(0.01)
        right_wam.set_goal_orientation_tolerance(0.05)
        left_wam.set_goal_position_tolerance(0.01)
        left_wam.set_goal_orientation_tolerance(0.05)

        # Allow replanning to increase the odds of a solution
        right_wam.allow_replanning(True)
        left_wam.allow_replanning(True)

        # Allow 5 seconds per planning attempt
        right_wam.set_planning_time(15)
        left_wam.set_planning_time(25)

        # Allow replanning to increase the odds of a solution
        right_wam.allow_replanning(True)
        left_wam.allow_replanning(True)
        
        
        # Set the reference frame for wam arms
        left_wam.set_pose_reference_frame(REFERENCE_FRAME)
        right_wam.set_pose_reference_frame(REFERENCE_FRAME)

        # Set a limit on the number of pick attempts before bailing
        max_pick_attempts = 5
        
        # Set a limit on the number of place attempts
        max_place_attempts = 5

        # Give the scene a chance to catch up
        rospy.sleep(2)

        # Give each of the scene objects a unique name        
        table_id = 'table'
        bowl_id = 'bowl'
        pitcher_id = 'pitcher'
        spoon_id = 'spoon'
 
        # Remove leftover objects from a previous run
        scene.remove_world_object(table_id)
        scene.remove_world_object(bowl_id)
        scene.remove_world_object(pitcher_id)
        scene.remove_world_object(spoon_id)

        # Remove leftover objects from a previous run
        scene.remove_attached_object(right_end_effector_link, 'spoon')

        #right_wam.set_named_target('right_wam_start')
        #right_wam.go()
        #rospy.sleep(2)
        # Closing the hand first
        # Closing the hand
        #right_wam_finger_1.set_named_target("right_wam_finger_1_grasp")
        #right_wam_finger_2.set_named_target("right_wam_finger_2_grasp")
        #right_wam_finger_3.set_named_target("right_wam_finger_3_grasp")
 
        #right_wam_finger_1.execute(right_wam_finger_1.plan())
        #rospy.sleep(5)   
        #right_wam_finger_2.execute(right_wam_finger_2.plan())
        #rospy.sleep(5)    
        #right_wam_finger_3.execute(right_wam_finger_3.plan())  
        #rospy.sleep(5)

        # Create a pose for the tool relative to the end-effector
        p = PoseStamped()
        p.header.frame_id = right_end_effector_link
        
        # Place the end of the object within the grasp of the gripper
        p.pose.position.x = 0.0
        p.pose.position.y = 0.0
        p.pose.position.z = -0.02
        
        # Align the object with the gripper (straight out)
        p.pose.orientation.x = -0.5
        p.pose.orientation.y = 0.5
        p.pose.orientation.z = -0.5
        p.pose.orientation.w = 0.5
        # Attach the tool to the end-effector
        

        # Set the height of the table off the ground
        table_ground = 0.5762625 
        
        # Set the length, width and height of the table and boxes
        table_size = [0.90128, 0.381, 0.0238125]

        table_pose = PoseStamped()
        table_pose.header.frame_id = REFERENCE_FRAME
        table_pose.pose.position.x = 0
        table_pose.pose.position.y = 0.847725
        table_pose.pose.position.z = table_ground
        scene.add_box(table_id, table_pose, table_size)

        # Set the height of the bowl
        bowl_ground = 0.57816875 
        bowl_pose = PoseStamped()
        bowl_pose.header.frame_id = REFERENCE_FRAME
        bowl_pose.pose.position.x = 0.015
        bowl_pose.pose.position.y = 0.847725
        bowl_pose.pose.position.z = bowl_ground
        scene.add_mesh(bowl_id, bowl_pose, '/home/yzheng/catkin_ws/src/manipulation_scenarios/ycb_object_models/models/stl/bowl.stl')

        # Set the height of the pitcher
        #pitcher_ground = 0.57816875 
        #pitcher_pose = PoseStamped()
        #pitcher_pose.header.frame_id = REFERENCE_FRAME
        #pitcher_pose.pose.position.x = 0.25
        #pitcher_pose.pose.position.y = 0.847725
        #pitcher_pose.pose.position.z = pitcher_ground
        #pitcher_pose.pose.orientation.w = -0.5
        #pitcher_pose.pose.orientation.z = 0.707
        #scene.add_mesh(pitcher_id, pitcher_pose, '/home/yzheng/catkin_ws/src/manipulation_scenarios/ycb_object_models/models/stl/pitcher.stl')

        # Make the table red and the boxes orange
        self.setColor(table_id, 0.8, 0.4, 0, 1.0)
        self.setColor(bowl_id, 0, 0.4, 0.8, 1.0)
        #self.setColor(pitcher_id, 0.9, 0.9, 0, 1.0)
        self.sendColors() 
        rospy.sleep(2) 

        start = input("Start left_wam planning ? ")

        # Set the support surface name to the table object
        #left_wam.set_support_surface_name(table_id)
        #right_wam.set_support_surface_name(table_id)

        # Set the target pose.
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.header.stamp = rospy.Time.now()  

        target_pose.pose.position.x = 0.40363476287
        target_pose.pose.position.y = 0.847725
        target_pose.pose.position.z = 0.721472317843
        target_pose.pose.orientation.x = 0.707
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = -0.707
        target_pose.pose.orientation.w = 0

        # Set the start state to the current state
        left_wam.set_start_state_to_current_state()
        
        # Set the goal pose of the end effector to the stored pose
        left_wam.set_pose_target(target_pose, left_end_effector_link)

        left_wam.execute(left_wam.plan())
        
        left_wam.shift_pose_target(5, 0, left_end_effector_link)


        start = input("Left wam starting position ")
        # Pause for a second
        # Closing the hand
        left_wam_finger_1.set_named_target("left_wam_finger_1_grasp")
        left_wam_finger_2.set_named_target("left_wam_finger_2_grasp")
        left_wam_finger_3.set_named_target("left_wam_finger_3_grasp")
 
        left_wam_finger_1.execute(left_wam_finger_1.plan())
        #rospy.sleep(3)   
        left_wam_finger_2.execute(left_wam_finger_2.plan())
        #rospy.sleep(3)    
        left_wam_finger_3.execute(left_wam_finger_3.plan())  
        #rospy.sleep(3)
       
        start = input("Left wam hand closing ")

        end_pose = deepcopy(left_wam.get_current_pose(left_end_effector_link).pose)
        intermidiate_pose = deepcopy(end_pose)
        intermidiate_pose.position.z = intermidiate_pose.position.z + 0.05

        plan = self.StraightPath(end_pose, intermidiate_pose, left_wam)
        left_wam.set_start_state_to_current_state()
        left_wam.execute(plan)
      
        start = input("Hold up the Pitcher ")
        end_pose = deepcopy(left_wam.get_current_pose(left_end_effector_link).pose)
        intermidiate_pose = deepcopy(end_pose)
        intermidiate_pose.position.x = intermidiate_pose.position.x - 0.1

        plan = self.StraightPath(end_pose, intermidiate_pose, left_wam)
        left_wam.set_start_state_to_current_state()
        left_wam.execute(plan)
        start = input("left_wam into pouring position ")

        end_pose = deepcopy(left_wam.get_current_pose(left_end_effector_link))
        back_pose = deepcopy(end_pose)
        end_pose.pose.orientation.x = 0.97773401145
        end_pose.pose.orientation.y = 0
        end_pose.pose.orientation.z = -0.209726592658
        end_pose.pose.orientation.w = 0
         
        # Set the start state to the current state
        left_wam.set_start_state_to_current_state()
        
        # Set the goal pose of the end effector to the stored pose
        left_wam.set_pose_target(end_pose, left_end_effector_link)

        left_wam.execute(left_wam.plan())
        start = input("Pour the water ")


        # Set the start state to the current state
        left_wam.set_start_state_to_current_state()
        
        # Set the goal pose of the end effector to the stored pose
        left_wam.set_pose_target(back_pose, left_end_effector_link)

        left_wam.execute(left_wam.plan())
 
        end_pose = deepcopy(left_wam.get_current_pose(left_end_effector_link))
        end_pose.pose.position.x = end_pose.pose.position.x + 0.1
        # Set the start state to the current state
        left_wam.set_start_state_to_current_state()
        
        # Set the goal pose of the end effector to the stored pose
        left_wam.set_pose_target(end_pose, left_end_effector_link)

        left_wam.execute(left_wam.plan())
        start = input("Left_wam back to prep position")
        
       

        

        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.header.stamp = rospy.Time.now()  

        target_pose.pose.position.x = -0.600350195463908
        target_pose.pose.position.y = 0.80576308041
        target_pose.pose.position.z = 0.794775212132
        target_pose.pose.orientation.x = 3.01203251908e-05
        target_pose.pose.orientation.y = 0.705562870053
        target_pose.pose.orientation.z = 4.55236739937e-05
        target_pose.pose.orientation.w = 0.708647326547

        start = input("Start right_wam planning ? ")
        right_wam.set_start_state_to_current_state()
        right_wam.set_pose_target(target_pose, right_end_effector_link)
        right_wam.execute(right_wam.plan())
    
        start = input("right_wam into position. ")  
        start_pose = deepcopy(right_wam.get_current_pose(right_end_effector_link).pose) 
        intermidiate_pose =  deepcopy(right_wam.get_current_pose(right_end_effector_link).pose)
        intermidiate_pose.position.x = intermidiate_pose.position.x + 0.6
        plan = self.StraightPath(start_pose, intermidiate_pose, right_wam)
        right_wam.set_start_state_to_current_state()
        right_wam.execute(plan) 

        rospy.sleep(3)
        
        # Closing the hand
        right_wam_finger_1.set_named_target("right_wam_finger_1_grasp")
        right_wam_finger_2.set_named_target("right_wam_finger_2_grasp")
        right_wam_finger_3.set_named_target("right_wam_finger_3_grasp")
 
        right_wam_finger_1.execute(right_wam_finger_1.plan())
        #rospy.sleep(3)   
        right_wam_finger_2.execute(right_wam_finger_2.plan())
        #rospy.sleep(3)    
        right_wam_finger_3.execute(right_wam_finger_3.plan())  
        rospy.sleep(1)
        scene.attach_mesh(right_end_effector_link, 'spoon', p, '/home/yzheng/catkin_ws/src/manipulation_scenarios/ycb_object_models/models/stl/spoon.stl')


        #create a circle path
        circles = input("How many circles you want the wam to mix ? ")
  
        start_pose = deepcopy(right_wam.get_current_pose(right_end_effector_link).pose)
        plan = self.CircularPath(start_pose, circles, right_wam)

        #execute the circle path
        right_wam.set_start_state_to_current_state()
        right_wam.execute(plan)

        pause = input("Mix the oatmeal ")

    
        #put the right_wam back to preparation pose
        end_pose = deepcopy(right_wam.get_current_pose(right_end_effector_link).pose)
        intermidiate_pose1 = deepcopy(end_pose)
        intermidiate_pose1.position.z = intermidiate_pose1.position.z + 0.1

        plan = self.StraightPath(end_pose, intermidiate_pose1, right_wam)
        right_wam.set_start_state_to_current_state()
        right_wam.execute(plan)

        pause = input("wait for the execution of straight path  ")

        end_pose = deepcopy(right_wam.get_current_pose(right_end_effector_link).pose)
        intermidiate_pose2 = deepcopy(end_pose)
        intermidiate_pose2.position.x = intermidiate_pose2.position.x - 0.25

        plan = self.StraightPath(end_pose, intermidiate_pose2, right_wam)
        right_wam.set_start_state_to_current_state()
        right_wam.execute(plan)
        
        pause = input("right_wam back into prep position  ")
             
        

        #left_wam.shift_pose_target(5, 0, left_end_effector_link)
        #left_wam.go()
        #rospy.sleep(2)

        #left_wam.shift_pose_target(0, -0.05, left_end_effector_link)
        #left_wam.go()
        #rospy.sleep(2)

        # Initialize the grasp pose to the target pose
        #grasp_pose = target_pose
        # Generate a list of grasps
        #grasps = self.make_grasps(grasp_pose, [pitcher_id])
    
        # Publish the grasp poses so they can be viewed in RViz
        #for grasp in grasps:
        #    self.left_wam_finger_1_pub.publish(grasp.grasp_pose)
        #    rospy.sleep(0.2)
 
        #    self.left_wam_finger_2_pub.publish(grasp.grasp_pose)
        #    rospy.sleep(0.2)

        #    self.left_wam_finger_3_pub.publish(grasp.grasp_pose)
        #    rospy.sleep(0.2)
 
        
        
       
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)



    # Generate a list of possible grasps
    #def make_grasps(self, initial_pose_stamped, allowed_touch_objects):
        # Initialize the grasp object
        #g_left_wam_finger_1 = Grasp()
        #g_left_wam_finger_2 = Grasp()
        #g_left_wam_finger_3 = Grasp()
        
        # Set the pre-grasp and grasp postures appropriately
        #g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        #g.grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)
                
        # Set the approach and retreat parameters as desired
        #g.pre_grasp_approach = self.make_gripper_translation(0.01, 0.1, [1.0, 0.0, 0.0])
        #g.post_grasp_retreat = self.make_gripper_translation(0.1, 0.15, [0.0, -1.0, 1.0])
        
        # Set the first grasp pose to the input pose
        #g.grasp_pose = initial_pose_stamped
    
        # Pitch angles to try
        #pitch_vals = [0, 0.1, -0.1, 0.2, -0.2, 0.3, -0.3]
        
        # Yaw angles to try
        #yaw_vals = [0]

        # A list to hold the grasps
        #grasps = []

        # Generate a grasp for each pitch and yaw angle
        #for y in yaw_vals:
        #    for p in pitch_vals:
                # Create a quaternion from the Euler angles
        #        q = quaternion_from_euler(0, p, y)
                
                # Set the grasp pose orientation accordingly
        #        g.grasp_pose.pose.orientation.x = q[0]
        #        g.grasp_pose.pose.orientation.y = q[1]
        #        g.grasp_pose.pose.orientation.z = q[2]
        #        g.grasp_pose.pose.orientation.w = q[3]
                
                # Set and id for this grasp (simply needs to be unique)
        #        g.id = str(len(grasps))
                
                # Set the allowed touch objects to the input list
        #        g.allowed_touch_objects = allowed_touch_objects
                
                # Don't restrict contact force
        #        g.max_contact_force = 0
                
                # Degrade grasp quality for increasing pitch angles
        #        g.grasp_quality = 1.0 - abs(p)
                
                # Append the grasp to the list
        #        grasps.append(deepcopy(g))
                
        # Return the list
        #return grasps

        

        # Set the color of an object
    def setColor(self, name, r, g, b, a = 0.9):
        # Initialize a MoveIt color object
        color = ObjectColor()
        
        # Set the id to the name given as an argument
        color.id = name
        
        # Set the rgb and alpha values given as input
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # Update the global color dictionary
        self.colors[name] = color

    # Actually send the colors to MoveIt!
    def sendColors(self):
        # Initialize a planning scene object
        p = PlanningScene()

        # Need to publish a planning scene diff        
        p.is_diff = True
        
        # Append the colors from the global color dictionary 
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # Publish the scene diff
        self.scene_pub.publish(p)

    # Function to execute a circular path
    def CircularPath(self, start_pose, numOfCircle, moveGroup):
        waypoints = []
        waypoints.append(start_pose)

        center_pose = deepcopy(start_pose)
        center_pose.position.y = start_pose.position.y + 0.04
        
        angle_resolution = 10
        d_angle = angle_resolution*3.14159/180
        angle = 0

        wpose = deepcopy(center_pose)
        for n in range(0, int(math.floor(numOfCircle * 360/angle_resolution)) + 1):
             angle = angle + d_angle
             wpose.position.x = center_pose.position.x + 0.03 * math.cos(angle)
             wpose.position.y = center_pose.position.y + 0.03 * math.sin(angle)
             waypoints.append(deepcopy(wpose))

        fraction = 0.0
        maxtries = 300
        attempts = 0

        # Set the internal state to the current state
        #right_wam.set_start_state_to_current_state()
     
        # Plan the Cartesian path connecting the waypoints
        while fraction < 1.0 and attempts < maxtries:
              (plan, fraction) = moveGroup.compute_cartesian_path(waypoints, 0.01, 0.0, True)       
                
              # Increment the number of attempts 
              attempts = attempts +  1
                
              # Print out a progress message
              if attempts % 10 == 0:
                    rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                         
               # If we have a complete plan, execute the trajectory
        if fraction == 1.0:
                rospy.loginfo("Path computed successfully. Moving the arm.")
    
                #right_wam.execute(plan)
                            
                rospy.loginfo("Path execution complete.")
                return plan
        else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.") 
     
    def StraightPath(self, start_pose, end_pose, moveGroup):
        waypoints = []
        waypoints.append(start_pose)
        waypoints.append(end_pose)
        # Plan the Cartesian path connecting the waypoints

        fraction = 0.0
        maxtries = 300
        attempts = 0

        while fraction < 1.0 and attempts < maxtries:
              (plan, fraction) = moveGroup.compute_cartesian_path(waypoints, 0.01, 0.0, True)       
                
              # Increment the number of attempts 
              attempts = attempts +  1
                
              # Print out a progress message
              if attempts % 10 == 0:
                    rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                         
               # If we have a complete plan, execute the trajectory
        if fraction == 1.0:
                rospy.loginfo("Path computed successfully. Moving the arm.")
    
                #right_wam.execute(plan)
                            
                rospy.loginfo("Path execution complete.")
                return plan
        else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.") 
   
       

if __name__ == '__main__':
    try:
        MotionSequence()
    except rospy.ROSInterruptException:
        rospy.loginfo("Planning publisher is shut down.")
