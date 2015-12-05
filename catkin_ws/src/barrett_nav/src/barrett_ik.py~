#!/usr/bin/env python

import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class TrajInverseKinematics:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('TrajForwardKinematics', anonymous=True)


        # Define move group comander for each moveit group
        left_wam = moveit_commander.MoveGroupCommander('left_wam')
        right_wam = moveit_commander.MoveGroupCommander('right_wam')
        left_wam_finger_1 = moveit_commander.MoveGroupCommander('left_wam_finger_1')
        left_wam_finger_2 = moveit_commander.MoveGroupCommander('left_wam_finger_2')
        left_wam_finger_3 = moveit_commander.MoveGroupCommander('left_wam_finger_3')
        right_wam_finger_1 = moveit_commander.MoveGroupCommander('right_wam_finger_1')
        right_wam_finger_2 = moveit_commander.MoveGroupCommander('right_wam_finger_2')
        right_wam_finger_3 = moveit_commander.MoveGroupCommander('right_wam_finger_3')

        # Get the name of the end-effector link
        end_effector_link = left_wam.get_end_effector_link()
        # Display the name of the end_effector link
        rospy.loginfo("The end effector link is: " + str(end_effector_link))

        # Set the reference frame for pose targets
        reference_frame = 'rack1'
        
        # Set the right arm reference frame accordingly
        left_wam.set_pose_reference_frame(reference_frame)
                
        # Allow replanning to increase the odds of a solution
        left_wam.allow_replanning(True)
        
        # Allow some leeway in position (meters) and orientation (radians)
        left_wam.set_goal_position_tolerance(0.01)
        left_wam.set_goal_orientation_tolerance(0.01)
        
        # Set the target pose.
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()  

        target_pose.pose.position.x = 0.164602723532
        target_pose.pose.position.y = 0.923276839804
        target_pose.pose.position.z = 0.755820679544
        target_pose.pose.orientation.x = -0.70865246992
        target_pose.pose.orientation.y = 1.2218133268e-05
        target_pose.pose.orientation.z = 0.705557706104
        target_pose.pose.orientation.w = 9.17788731987e-06

        # Set the start state to the current state
        left_wam.set_start_state_to_current_state()
        
        # Set the goal pose of the end effector to the stored pose
        left_wam.set_pose_target(target_pose, end_effector_link)

        left_wam.execute(left_wam.plan())
        # Pause for a second
        rospy.sleep(2)
##########################################################################################
        target_pose.pose.position.x = 0.0852304558828
        target_pose.pose.position.y = 0.923190555814
        target_pose.pose.position.z = 0.755820679544
        target_pose.pose.orientation.x = 0.708563141475
        target_pose.pose.orientation.y = 2.51706911305e-05
        target_pose.pose.orientation.z = -0.705647413675
        target_pose.pose.orientation.w = 3.85066092542e-05

        # Set the start state to the current state
        #left_wam.set_start_state_to_current_state()
        
        # Set the goal pose of the end effector to the stored pose
        left_wam.set_pose_target(target_pose, end_effector_link)

        left_wam.execute(left_wam.plan())
        # Pause for a second
        rospy.sleep(4)

        left_wam.shift_pose_target(5, 0, end_effector_link)
        left_wam.go()
        rospy.sleep(2)
#################################################################################
        # Closing the hand
        left_wam_finger_1.set_named_target("left_wam_finger_1_grasp")
        left_wam_finger_2.set_named_target("left_wam_finger_2_grasp")
        left_wam_finger_3.set_named_target("left_wam_finger_3_grasp")
 
        left_wam_finger_1.execute(left_wam_finger_1.plan())
        rospy.sleep(5)   
        left_wam_finger_2.execute(left_wam_finger_2.plan())
        rospy.sleep(5)    
        left_wam_finger_3.execute(left_wam_finger_3.plan())  
        rospy.sleep(5)
#################################################################################
        target_pose.pose.position.x = 0.0843395169092
        target_pose.pose.position.y = 0.923283112735
        target_pose.pose.position.z = 0.960625217159
        target_pose.pose.orientation.x = 0.708619656094
        target_pose.pose.orientation.y = -1.02875262203e-07
        target_pose.pose.orientation.z = -0.705590662304
        target_pose.pose.orientation.w = 1.63147849371e-05

        # Set the start state to the current state
        #left_wam.set_start_state_to_current_state()
        
        # Set the goal pose of the end effector to the stored pose
        left_wam.set_pose_target(target_pose, end_effector_link)

        left_wam.execute(left_wam.plan())
        # Pause for a second
        rospy.sleep(4)
###################################################################################
        target_pose.pose.position.x = -0.000191779497579
        target_pose.pose.position.y = 0.923305328478
        target_pose.pose.position.z = 0.960222944087
        target_pose.pose.orientation.x = -0.708696720695
        target_pose.pose.orientation.y = 4.27715484909e-05
        target_pose.pose.orientation.z = 0.705513256626
        target_pose.pose.orientation.w = 3.11677563983e-05

        # Set the goal pose of the end effector to the stored pose
        left_wam.set_pose_target(target_pose, end_effector_link)

        left_wam.execute(left_wam.plan())
        # Pause for a second
        rospy.sleep(4)

        left_wam.shift_pose_target(5, 0, end_effector_link)
        left_wam.go()
        rospy.sleep(2)
##################################################################################
        target_pose.pose.position.x = -0.0426697673797
        target_pose.pose.position.y = 0.923257650909
        target_pose.pose.position.z = 0.868823611737
        target_pose.pose.orientation.x = 0.939606018038
        target_pose.pose.orientation.y = -5.8074608327e-05
        target_pose.pose.orientation.z = -0.342257975782 
        target_pose.pose.orientation.w = 7.42148960716e-05

        # Set the goal pose of the end effector to the stored pose
        left_wam.set_pose_target(target_pose, end_effector_link)

        left_wam.execute(left_wam.plan())
        # Pause for a second
        rospy.sleep(4)
##################################################################################
        target_pose.pose.position.x = -0.000191779497579
        target_pose.pose.position.y = 0.923305328478
        target_pose.pose.position.z = 0.960222944087
        target_pose.pose.orientation.x = -0.708696720695
        target_pose.pose.orientation.y = 4.27715484909e-05
        target_pose.pose.orientation.z = 0.705513256626
        target_pose.pose.orientation.w = 3.11677563983e-05

        # Set the goal pose of the end effector to the stored pose
        left_wam.set_pose_target(target_pose, end_effector_link)

        left_wam.execute(left_wam.plan())
        # Pause for a second
        rospy.sleep(4)
        left_wam.shift_pose_target(5, 0, end_effector_link)
        left_wam.go()
        rospy.sleep(2)
####################################################################################
        target_pose.pose.position.x = 0.0843395169092
        target_pose.pose.position.y = 0.923283112735
        target_pose.pose.position.z = 0.960625217159
        target_pose.pose.orientation.x = 0.708619656094
        target_pose.pose.orientation.y = -1.02875262203e-07
        target_pose.pose.orientation.z = -0.705590662304
        target_pose.pose.orientation.w = 1.63147849371e-05

        # Set the start state to the current state
        #left_wam.set_start_state_to_current_state()
        
        # Set the goal pose of the end effector to the stored pose
        left_wam.set_pose_target(target_pose, end_effector_link)

        left_wam.execute(left_wam.plan())
        # Pause for a second
        rospy.sleep(2)
#####################################################################################

        target_pose.pose.position.x = 0.0852304558828
        target_pose.pose.position.y = 0.923190555814
        target_pose.pose.position.z = 0.755820679544
        target_pose.pose.orientation.x = 0.708563141475
        target_pose.pose.orientation.y = 2.51706911305e-05
        target_pose.pose.orientation.z = -0.705647413675
        target_pose.pose.orientation.w = 3.85066092542e-05

        # Set the start state to the current state
        #left_wam.set_start_state_to_current_state()
        
        # Set the goal pose of the end effector to the stored pose
        left_wam.set_pose_target(target_pose, end_effector_link)

        left_wam.execute(left_wam.plan())
        # Pause for a second
        rospy.sleep(4)
######################################################################################
        # open the hand
        left_wam_finger_1.set_named_target("left_wam_finger_1_home")
        left_wam_finger_2.set_named_target("left_wam_finger_2_home")
        left_wam_finger_3.set_named_target("left_wam_finger_3_home")
 
        left_wam_finger_1.execute(left_wam_finger_1.plan())
        rospy.sleep(5)   
        left_wam_finger_2.execute(left_wam_finger_2.plan())
        rospy.sleep(5)    
        left_wam_finger_3.execute(left_wam_finger_3.plan()) 
        rospy.sleep(5)
###################################################################################### 
# Set the target pose.
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()  

        target_pose.pose.position.x = 0.164602723532
        target_pose.pose.position.y = 0.923276839804
        target_pose.pose.position.z = 0.755820679544
        target_pose.pose.orientation.x = -0.70865246992
        target_pose.pose.orientation.y = 1.2218133268e-05
        target_pose.pose.orientation.z = 0.705557706104
        target_pose.pose.orientation.w = 9.17788731987e-06

        # Set the start state to the current state
        #left_wam.set_start_state_to_current_state()
        
        # Set the goal pose of the end effector to the stored pose
        left_wam.set_pose_target(target_pose, end_effector_link)

        left_wam.execute(left_wam.plan())
        # Pause for a second
        rospy.sleep(3)
##################################################################################
        left_wam.set_named_target('left_wam_home')
        left_wam.go()
        rospy.sleep(1)

        # Cleanly shut down MoveIt
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        TrajInverseKinematics()
    except rospy.ROSInterruptException:
        pass




# x: 0.0843395169092 y: 0.923283112735  z: 0.960625217159
# x: 0.708619656094  y: -1.02875262203e-07   z: -0.705590662304  w: 1.63147849371e-05
#RPY = [3.136161073444429, 1.5665126164981247, -0.005408652990239754]

#x: -0.000191779497579  y: 0.923305328478 z: 0.960222944087
#x: -0.708696720695  y: 4.27715484909e-05  z: 0.705513256626   w: 3.11677563983e-05 
# RPY = [3.137999932255037, 1.5662941950225944, -0.0036972875675046137]

# x: 0.0426697673797   y: 0.923257650909  z: 0.868823611737
# x: 0.939606018038   y: -5.8074608327e-05  z: -0.342257975782  w: 7.42148960716e-05
#RPY = [3.1413586009786765, 0.6986379006659567, -0.00020887008345195563]
