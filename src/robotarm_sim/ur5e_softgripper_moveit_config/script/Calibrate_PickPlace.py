#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
import math
from math import pi
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from detection_msgs.msg import Detection2DArray, Detection2D
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler

class TransformPose:
	def __init__(self):
		#initialise moveit_commander and a rospy node
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('PickPlace_Node', anonymous=True)

		#instantiate a RobotCommander object
		robot = moveit_commander.RobotCommander()

		#instantitate a PlanningSceneInterface object
		scene = moveit_commander.PlanningSceneInterface()

		#instantitate a MoveGroupCommander object
		self.arm_group = moveit_commander.MoveGroupCommander("manipulator")
		self.hand_group = moveit_commander.MoveGroupCommander("gripper_finger")

		self.arm_group.set_num_planning_attempts(20)
		# self.arm_group.set_planning_time(10)

		#Create DisplayTrajectory ROS publisher which is used to display trajectories in Rviz
		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

		self.tf_buffer = tf2_ros.Buffer()
		tf_listener = tf2_ros.TransformListener(self.tf_buffer)

		#Subscribe to object pose
		self.object_pose = rospy.Subscriber('/yolov5/detections', Detection2DArray, self.detection_callback)

		#Subscribe to plate pose
		self.plate_pose_sub = rospy.Subscriber('/pose/red_object', Detection2DArray, self.pos_callback)

	def pos_callback(self, plate_data):

		for detection in plate_data.detections:
			
			plate_pose = Detection2D()
			plate_pose.header.stamp = detection.header.stamp
			plate_pose.header.frame_id = detection.header.frame_id

			#Unused in the code
			plate_pose.bbox = detection.bbox 
			plate_pose.results.score = detection.results.score

			plate_pose.results.Class = detection.results.Class
			# plate_pose.results.pose = detection.results.pose

			plate_pose_stamped = PoseStamped()
			plate_pose_stamped.pose = detection.results.pose
			plate_pose_stamped.header.stamp = detection.header.stamp
			plate_pose_stamped.header.frame_id = detection.header.frame_id

		try:
			camera_to_robot_transform_stamped = self.tf_buffer.lookup_transform("base_link", "camera_depth_optical_frame", rospy.Time())
			self.plate_pose = tf2_geometry_msgs.do_transform_pose(plate_pose_stamped, camera_to_robot_transform_stamped)

			# print("Received object pose wrt to robot base", plate_pose)
			# print(camera_to_robot_transform_stamped)
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			rospy.logerr("Failed to transform object pose from camera frame to robot's frame")

	def detection_callback(self, Detection2DArray_data):
		
		detected_obj_list = []

		for detection in Detection2DArray_data.detections:

			detected_obj = Detection2D()
			detected_obj.header.stamp = detection.header.stamp
			detected_obj.header.frame_id = detection.header.frame_id
			
			detected_obj.bbox = detection.bbox

			detected_obj.results.Class = detection.results.Class
			detected_obj.results.score = detection.results.score

			detected_obj.results.pose = detection.results.pose

			detected_pose_stamped = PoseStamped()
			detected_pose_stamped.header.stamp = detected_obj.header.stamp
			detected_pose_stamped.header.frame_id = detected_obj.header.frame_id

			detected_pose_stamped.pose = detected_obj.results.pose

			try:
				camera_to_robot_transform_stamped = self.tf_buffer.lookup_transform("base_link", "camera_depth_optical_frame", rospy.Time())
				object_pose = tf2_geometry_msgs.do_transform_pose(detected_pose_stamped, camera_to_robot_transform_stamped)

				detected_obj.results.pose = object_pose.pose

				detected_obj_list.append(detected_obj)

				# print("Received object pose wrt to robot base", plate_pose)
				# print(camera_to_robot_transform_stamped)
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				rospy.logerr("Failed to transform object pose from camera frame to robot's frame")

		self.detected_obj_list = detected_obj_list

class PickPlace:
	def __init__(self):
		self.tp = TransformPose()

	def HomePosition(self):
		print(">> Going to Home Position")
		self.tp.arm_group.set_named_target("HomePosition")
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

	def SourceTable(self):
		#Go to above source table
		source = [-3.752, -1.431, 0.960, -1.1 ,-pi/2, -0.593]

		print(">> Going above source table")
		self.tp.arm_group.set_joint_value_target(source)
		self.tp.arm_group.go(wait=True)

		print(self.tp.arm_group.get_current_pose())

		print(">> Sleeping for 23 seconds for YOLO to catch up")
		rospy.sleep(23)

	def SourceTable_Object(self):

		source_object = geometry_msgs.msg.Pose()


		for obj in self.tp.detected_obj_list:
			if obj.results.Class == "red_cube":
				source_object.position.x = obj.results.pose.position.x
				source_object.position.y = obj.results.pose.position.y
				source_object.position.z = 1.1

				source_object.orientation.x = (obj.results.pose.orientation.x)
				source_object.orientation.y = (obj.results.pose.orientation.y)
				source_object.orientation.z = (obj.results.pose.orientation.z)
				source_object.orientation.w = (obj.results.pose.orientation.w)

				break

		# print(self.tp.object_pose.pose.orientation)
		# print(source_object.orientation)

		rospy.sleep(1)

		print(">> Going above detected object")
		print(source_object)
		self.tp.arm_group.set_pose_target(source_object)
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)


	def Pick(self):

		deflate = [0.087, 0.087, 0.087, 0.087] #5degrees

		print(">> Deflate gripper")
		self.tp.hand_group.set_joint_value_target(deflate)
		self.tp.hand_group.go()

		#Move towards object in z-direction
		print(">> Moving 0.2 meters downwards along the z-axis")
		pickpose = self.tp.arm_group.get_current_pose().pose
		pickpose.position.z -= 0.2
		self.tp.arm_group.set_pose_target(pickpose)
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

		# half_closed = [-0.227, -0.227, -0.227, -227] #-13 degrees
		# half_closed = [-0.192, -0.192, -0.192, -0.192] #-11 degrees
		# half_closed = [-0.157, -0.157, -0.157, -0.157] #-9degrees
		# half_closed = [-0.122, -0.122, -0.122, -0.122] #-7degrees
		half_closed = [-0.105, -0.105, -0.105, -0.105] #-6degrees
		# half_closed = [-0.087, -0.087, -0.087, -0.087] #-5degrees

		print(">> Closing gripper")
		self.tp.hand_group.set_joint_value_target(half_closed)
		self.tp.hand_group.go()

		rospy.sleep(1)

		#Move upwards in z-direction
		print(">> Moving 0.2 meters upwards along the z-axis")
		pickpose = self.tp.arm_group.get_current_pose().pose
		pickpose.position.z += 0.2

		self.tp.arm_group.set_pose_target(pickpose)
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

	def AssemblyTable(self):
		#Move above assembly table
		print(">> Going above assembly table")
		assembly = [-pi/2,-pi/2, 1.466,-1.431, -pi/2, 0]
		self.tp.arm_group.set_joint_value_target(assembly)
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

	def AssemblyTable_Object(self):
		#Move above plate on assembly table (Replace with camera)
		assembly_plate = geometry_msgs.msg.Pose()

		quartenion = quaternion_from_euler(0, pi, 0)
		assembly_plate.orientation.x = quartenion[0]
		assembly_plate.orientation.y = quartenion[1]
		assembly_plate.orientation.z = quartenion[2]
		assembly_plate.orientation.w = quartenion[3]
		
		assembly_plate.position.x = self.tp.plate_pose.pose.position.x
		assembly_plate.position.y = self.tp.plate_pose.pose.position.y
		assembly_plate.position.z = 1.1

		print(">> Going above detected plate on assembly table")
		self.tp.arm_group.set_pose_target(assembly_plate)
		print(assembly_plate)
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

		# print(">> Making final adjustments")
		# assembly_plate.position.x= self.tp.plate_pose.pose.position.x
		# assembly_plate.position.y= self.tp.plate_pose.pose.position.y
		# print(assembly_plate)

		# self.tp.arm_group.set_pose_target(assembly_plate)
		# self.tp.arm_group.go(wait=True)

		# rospy.sleep(1)

	def Place(self):
		#Move towards object in z-direction
		print(">> Moving 0.15 meters downpwards along the z-axis")
		placepose = self.tp.arm_group.get_current_pose().pose
		placepose.position.z -= 0.15
		self.tp.arm_group.set_pose_target(placepose)
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

		print(">> Opening gripper")
		self.tp.hand_group.set_named_target("Open")
		self.tp.hand_group.go()

		rospy.sleep(1)

		#Move upwards in z-direction
		print(">> Moving 0.20 meters upwards along the z-axis")
		placepose = self.tp.arm_group.get_current_pose().pose
		placepose.position.z += 0.15
		self.tp.arm_group.set_pose_target(placepose)
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

		self.tp.arm_group.clear_path_constraints()

if __name__ == '__main__':
	pickplace_object = PickPlace()
	pickplace_object.HomePosition()
	pickplace_object.SourceTable()
	pickplace_object.SourceTable_Object()
	pickplace_object.Pick()
	pickplace_object.AssemblyTable()
	pickplace_object.AssemblyTable_Object()
	pickplace_object.Place()
	pickplace_object.HomePosition()
