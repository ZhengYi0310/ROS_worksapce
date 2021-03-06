#!/usr/bin/env python



import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose
import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import  PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

class SceneSetup():
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('SceneSetup')
        
        # Construct the initial scene object
        scene = PlanningSceneInterface()
        
        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene)
     
        # Create a dictionary to hold object colors
        self.colors = dict()
        
        # Pause for the scene to get ready
        rospy.sleep(1)

        # Set the reference frame for pose targets
        reference_frame = 'rack1'
        table_id = 'table'
        bowl_id = 'bowl'
        pitcher_id = 'pitcher'
  
        # Remove leftover objects from a previous run
        scene.remove_world_object(table_id)
        scene.remove_world_object(bowl_id)
        scene.remove_world_object(pitcher_id)

        # Set the height of the table off the ground
        table_ground = 0.5762625 
        
        # Set the length, width and height of the table and boxes
        table_size = [1.0128, 0.481, 0.0238125]

        table_pose = PoseStamped()
        table_pose.header.frame_id = reference_frame
        table_pose.pose.position.x = 0
        table_pose.pose.position.y = 0.847725
        table_pose.pose.position.z = table_ground

        # Set the height of the bowl
        bowl_ground = 0.57816875 
        bowl_pose = PoseStamped()
        bowl_pose.header.frame_id = reference_frame
        bowl_pose.pose.position.x = 0
        bowl_pose.pose.position.y = 0.847725
        bowl_pose.pose.position.z = bowl_ground

        # Set the height of the pitcher
        pitcher_ground = 0.57816875 
        pitcher_pose = PoseStamped()
        pitcher_pose.header.frame_id = reference_frame
        pitcher_pose.pose.position.x = 0.4
        pitcher_pose.pose.position.y = 0.847725
        pitcher_pose.pose.position.z = pitcher_ground
        pitcher_pose.pose.orientation.w = -0.5
        pitcher_pose.pose.orientation.z = 0.707

        # Make the table red and the boxes orange
        self.setColor(table_id, 0.8, 0.4, 0, 1.0)
        self.setColor(bowl_id, 0, 0.4, 0.8, 1.0)
        self.setColor(pitcher_id, 0, 0.4, 0.8, 1.0)
        self.sendColors()   


        
        scene.add_box(table_id, table_pose, table_size)
        scene.add_mesh(bowl_id, bowl_pose, '/home/yzheng/catkin_ws/src/manipulation_scenarios/ycb_object_models/models/stl/bowl.stl')
               #scene.add_mesh(pitcher_id, pitcher_pose, '/home/yzheng/catkin_ws/src/manipulation_scenarios/ycb_object_models/models/stl/pitcher.stl')

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

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

if __name__ == '__main__':
    try:
        target = SceneSetup()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Planning scene publisher is shut down.")
