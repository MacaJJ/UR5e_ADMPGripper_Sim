#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import quaternion_from_euler
from math import pi
from detection_msgs.msg import Detection2DArray, Detection2D

class DepthCamera:
	def __init__(self):
		rospy.init_node('Plate_Detector')

		# Initialize CvBridge
		self.bridge = CvBridge()

		# Subscriber for the color image
		self.color_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)

		# Subscriber for the depth image
		self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)

		# Publisher for output image
		self.image_pub = rospy.Publisher('/output_image', Image, queue_size=1)

		# Publisher for object pose
		self.object_pub = rospy.Publisher('/pose/red_object', Detection2DArray, queue_size=1)

		# Subscriber for depth camera's intrinsic parameters
		camera_info = rospy.wait_for_message('/camera/depth/camera_info', CameraInfo)
		self.fx = camera_info.K[0]
		self.fy = camera_info.K[4]
		self.pp_cx = camera_info.K[2]
		self.pp_cy = camera_info.K[5]

	def color_callback(self, color_data):
		try:
			color_image = self.bridge.imgmsg_to_cv2(color_data, "bgr8")
		except CvBridgeError as e:
			print(e)

		lower_red = np.array([0, 0, 60], dtype="uint8")
		upper_red = np.array([100, 50, 255], dtype="uint8")
		mask = cv2.inRange(color_image, lower_red, upper_red)
		output = cv2.bitwise_and(color_image, color_image, mask=mask)

		kernel = np.ones((3,3),np.uint8)
		erosion = cv2.erode(output, kernel, iterations = 4)
		dilate = cv2.dilate(erosion, kernel)

		gray = cv2.cvtColor(dilate, cv2.COLOR_BGR2GRAY)

		self.gray = gray

		contours, hierarchy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]

		for cnt in contours:
			cv2.drawContours(color_image, [cnt], -1, (255,0,0), 3)
			M = cv2.moments(cnt)
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			cv2.circle(color_image,(cx,cy), 5, (255,0,255), -1)

		self.image_pub.publish(self.bridge.cv2_to_imgmsg(color_image, "bgr8"))

	def depth_callback(self, depth_data):
		try:
			depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
		except CvBridgeError as e:
			print(e)

		if self.gray.any():
			contours, hierarchy = cv2.findContours(self.gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]

			object_array = Detection2DArray()
			object_array.header.stamp = rospy.Time.now()
			object_array.header.frame_id = "camera_depth_optical_frame"

			for cnt in contours:
				M = cv2.moments(cnt)
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])

				object_pose = Detection2D()
				object_pose.header.stamp = rospy.Time.now()
				object_pose.header.frame_id = "camera_depth_optical_frame"

				#Bounding box not present for this code but is needed for Detection2D
				object_pose.bbox.center.x = 0
				object_pose.bbox.center.y = 0
				object_pose.bbox.center.theta = 0
				object_pose.results.score = 1

				# if area < 1000:
				# 	continue #ignoring small contours
				# elif area < 4000:
				# 	object_pose.results.Class = "cup"
				# elif area < 10000:
				# 	object_pose.results.Class = "bowl"
				# else:
				# 	object_pose.results.Class = "plate"

				if cv2.contourArea(cnt) < 1000:
					continue
				else:
					object_pose.results.Class = "plate"

				contour_pose = Pose()
			
				depth = depth_image[cy, cx]/1000

				x_meters = (cx - self.pp_cx) * depth / self.fx
				y_meters = (cy - self.pp_cy) * depth / self.fy
				z_meters = depth

				if x_meters < -0.117:
					x_meters -= 0.025
				elif x_meters > 0.117:
					x_meters += 0.025
				
				if y_meters < -0.067:
					y_meters -= 0.015
				elif y_meters > 0.067: 
					y_meters += 0.015

				contour_pose.position.x = x_meters
				contour_pose.position.y = y_meters
				contour_pose.position.z = z_meters

				contour_pose_quaternion = quaternion_from_euler(0, 0, 0)
				contour_pose.orientation.x = contour_pose_quaternion[0]
				contour_pose.orientation.y = contour_pose_quaternion[1]
				contour_pose.orientation.z = contour_pose_quaternion[2]
				contour_pose.orientation.w = contour_pose_quaternion[3]

				object_pose.results.pose = contour_pose

				object_array.detections.append(object_pose)

				# print(object_pose)

			self.object_pub.publish(object_array)
				
if __name__ == '__main__':
	try:
		depth_camera = DepthCamera()
		rospy.spin()
	except KeyboardInterrupt:
		print('Shutting down the ROS Plate Detector Node')
		cv2.destroyAllWindows()