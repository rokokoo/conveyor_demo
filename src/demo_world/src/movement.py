#!/usr/bin/env python

import sys, rospy, moveit_commander
import moveit_msgs.msg, geometry_msgs.msg
import math, tf, numpy as np, rospkg
import cv2, cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from demo_world.msg import Tracker
from copy import deepcopy
from elfin_vision import elfin_vision

class elfin_mover(object):
	def __init__(self, group_name):
		moveit_commander.roscpp_initialize(sys.argv)

		rospy.on_shutdown(self.cleanup)

		# DEFINE MOVEIT PARAMETERS
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.move_group = moveit_commander.MoveGroupCommander(group_name)
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
		self.planning_frame = self.move_group.get_planning_frame()
		self.move_group.set_pose_reference_frame("elfin_base_link")
		self.eef_link = self.move_group.get_end_effector_link()
		self.group_names = self.robot.get_group_names()

		self.move_group.allow_looking(True)
		self.move_group.allow_replanning(True)

		# Tolerance for arm position (meters) and orientations (radians)
		self.move_group.set_goal_position_tolerance(0)
		self.move_group.set_goal_orientation_tolerance(0)
		self.move_group.set_planning_time(0.1)

		# Change relative speed/acceleration of joints
		# self.move_group.set_max_acceleration_scaling_factor(0.01)
		# self.move_group.set_max_velocity_scaling_factor(0.02)

		# DEFINE TRACKING COMPONENTS
		self.tracker = Tracker()

		self.cxy_sub = rospy.Subscriber('cxy', Tracker, self.tracking_func, queue_size=1)
		self.cxy_pub = rospy.Publisher('cxy1', Tracker,queue_size=1)

		self.phase = 1
		self.object_cnt = 0
		self.track_flag = False
		self.default_pose_flag = True

		self.cx = 960
		self.cy = 540

		self.points=[]
		self.state_change_time = rospy.Time.now()

		self.waypoints = []
		self.pointx = []
		self.pointy = []

		# Default joint values, aka home position
		self.default_joint_states = self.move_group.get_current_joint_values()
		self.default_joint_states[0] = 0
		self.default_joint_states[1] = 0
		self.default_joint_states[2] = np.deg2rad(45)
		self.default_joint_states[3] = 0
		self.default_joint_states[4] = np.deg2rad(135)
		self.default_joint_states[5] = 0

		self.move_group.set_joint_value_target(self.default_joint_states)
		self.waypoints.append(self.move_group.get_current_pose(self.eef_link).pose)
		self.default_pose = self.move_group.get_current_pose(self.eef_link).pose
		plan = self.move_group.plan()

		self.move_group.execute(plan)

		self.move_group.set_start_state_to_current_state()

		#print("[0.024443427889326408, 0.03248160667721933, 0.7572031096693292, -0.10376988086007088, 2.361818020322353, -0.1385911134914073]")
		# Drop point
		self.drop_point = deepcopy(self.default_joint_states)
		self.drop_point[0] = np.deg2rad(180)
		self.drop_point[1] = np.deg2rad(15)
		self.drop_point[2] = np.deg2rad(124)
		self.drop_point[4] = np.deg2rad(71)

		# Trajectory
		# [0.020344523097919343, 0.3526061793557771, 1.5881158059204363, 0.028584035527498663, 1.9023478780300334, 0.08667419612412885]
		self.transition_pose = deepcopy(self.default_joint_states)

		self.mask = 1


	def tracking_func(self,msg):
		self.waypoints = []
		self.track_flag = msg.flag1
		self.cx = msg.x
		self.cy = msg.y
		self.error_x = msg.error_x
		self.error_y = msg.error_y
		# print(abs(self.error_x))
		# print(abs(self.error_y))
		# print("----------")

		## Go to tracked object
		

		if (abs(self.error_x)>2 or abs(self.error_y)>2) and self.track_flag:
			self.default_pose_flag = False
			self.goto_object()
		elif (abs(self.error_x)<=5 and abs(self.error_y)<=5) and self.track_flag:
			if self.mask == 1 and msg.red == 255:
				print("Mask set to green")
				ev.setMask(0,255,0)
				self.mask = 2

			elif self.mask == 2 and msg.green == 255:
				print("Mask set to blue")
				ev.setMask(255,0,0)
				self.mask = 3

			elif self.mask == 3 and msg.blue == 255:
				print("Mask set to red")
				ev.setMask(0,0,255)
				self.mask = 1
		
		if not self.default_pose_flag and not self.track_flag:
			self.track_flag = False
			self.goHome()

	def goHome(self):
		
		self.default_joint_states = self.move_group.get_current_joint_values()
		self.default_joint_states[0] = 0
		self.default_joint_states[1] = 0
		self.default_joint_states[2] = np.deg2rad(45)
		self.default_joint_states[3] = 0
		self.default_joint_states[4] = np.deg2rad(135)
		self.default_joint_states[5] = 0

		self.move_group.set_joint_value_target(self.default_joint_states)
		plan = self.move_group.plan()

		self.move_group.execute(plan, wait=True)

		self.default_pose_flag = True

	def add_obstacles(self):

		rospack = rospkg.RosPack()
		pkg_path = rospack.get_path('demo_world')

		p = geometry_msgs.msg.PoseStamped()
		p.header.frame_id = self.planning_frame

		p.pose.position.x = 0
		p.pose.position.y = 0
		p.pose.position.z = -0.5
		
		self.scene.add_plane("floor", p)

		p.pose.position.x = 0.7
		p.pose.position.y = 0
		p.pose.position.z = -0.5
		q = tf.transformations.quaternion_from_euler(np.deg2rad(90),0,np.deg2rad(90))
		p.pose.orientation.x = q[0]
		p.pose.orientation.y = q[1]
		p.pose.orientation.z = q[2]
		p.pose.orientation.w = q[3]
		
		p_name = "conveyor_belt"
		filename = pkg_path+"/meshes/conveyor_simple.stl"
		self.scene.add_mesh(p_name, p, filename, size=(1,1,1))

	def cleanup(self):
		rospy.loginfo("Stopping the robot")

		self.move_group.stop()

		rospy.loginfo("Shutting down Moveit!")
		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)

	def goto_object(self):
		# Track flag true, if camera sees desired object
		if self.track_flag:
			
			# Get the current pose for manipulation.
			wpose = self.move_group.get_current_pose(self.eef_link).pose
			# Initialize waypoint list
			self.waypoints = []
			
			# POSITION ON X AXIS
			wpose.position.x -= self.error_y*0.015/105
			# POSITION ON Y AXIS
			wpose.position.y -= self.error_x*0.015/105
			wpose.position.z = 0.5
			
			self.move_group.set_start_state_to_current_state()
			self.waypoints.append(deepcopy(wpose))

			plan, fraction = self.move_group.compute_cartesian_path(self.waypoints, 1, 0, True)

			# Move to target, if the used path is at least 80%
			if 1-fraction < 0.2:
				self.move_group.execute(plan, True)
		else:
			rospy.loginfo("No objects yet")

if __name__ == "__main__":
	rospy.init_node('elfin_mover', anonymous=False)
	em = elfin_mover("elfin_arm")
	ev = elfin_vision(255,0,0)
	rospy.spin()

# <group_state name="home" group="elfin_arm">
#         <joint name="elfin_joint1" value="0" />
#         <joint name="elfin_joint2" value="1.597" />
#         <joint name="elfin_joint3" value="2.61" />
#         <joint name="elfin_joint4" value="0" />
#         <joint name="elfin_joint5" value="-2.56" />
#         <joint name="elfin_joint6" value="0" />
#     </group_state>

 # -0.0074867022751 -0.103442825818 1.03643627725 -0.000172097821113 0.433533611123 5.61834322488e-05