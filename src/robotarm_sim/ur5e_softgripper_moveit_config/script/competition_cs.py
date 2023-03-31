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
		rospy.init_node('colour_segmentation')

		# Initialize CvBridge
		self.bridge = CvBridge()

		# Subscriber for the color image to isolate red
		self.color_sub_red = rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)

		# Subscriber for the depth image
		self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)

		# Publisher for output image
		self.image_pub = rospy.Publisher('/output_image', Image, queue_size=1)

		# Publisher for red mask
		self.red_mask_pub = rospy.Publisher('/mask/red', Image, queue_size=1)

		# Publisher for blue mask
		self.blue_mask_pub = rospy.Publisher('/mask/blue', Image, queue_size=1)

		# Publisher for object pose
		self.red_object_pub = rospy.Publisher('/pose_list/red', Detection2DArray, queue_size=1)

		# Publisher for object pose
		self.blue_object_pub = rospy.Publisher('/pose_list/blue', Detection2DArray, queue_size=1)

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

		color_image_copy = color_image.copy()

		#Isolate red objects
		lower_red = np.array([0, 0, 60], dtype="uint8")
		upper_red = np.array([100, 50, 255], dtype="uint8")
		mask = cv2.inRange(color_image_copy, lower_red, upper_red)
		output = cv2.bitwise_and(color_image_copy, color_image_copy, mask=mask)

		kernel = np.ones((3,3),np.uint8)
		erosion = cv2.erode(output, kernel, iterations = 4)
		dilate = cv2.dilate(erosion, kernel)

		red_gray = cv2.cvtColor(dilate, cv2.COLOR_BGR2GRAY)

		self.red_mask_pub.publish(self.bridge.cv2_to_imgmsg(dilate, encoding="bgr8"))

		self.red_gray = red_gray

		contours, hierarchy = cv2.findContours(red_gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]

		for cnt in contours:
			cv2.drawContours(color_image, [cnt], -1, (255,0,0), 3)
			M = cv2.moments(cnt)
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			cv2.circle(color_image,(cx,cy), 5, (255,0,0), -1)

		#Isolate blue objects
		lower_blue = np.array([100, 0, 0], dtype="uint8")
		upper_blue = np.array([255, 100, 100], dtype="uint8")
		mask = cv2.inRange(color_image_copy, lower_blue, upper_blue)
		output = cv2.bitwise_and(color_image_copy, color_image_copy, mask=mask)

		kernel = np.ones((3,3),np.uint8)
		erosion = cv2.erode(output, kernel, iterations = 4)
		dilate = cv2.dilate(erosion, kernel)

		blue_gray = cv2.cvtColor(dilate, cv2.COLOR_BGR2GRAY)

		self.blue_mask_pub.publish(self.bridge.cv2_to_imgmsg(dilate, encoding="bgr8"))

		self.blue_gray = blue_gray

		contours, hierarchy = cv2.findContours(blue_gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]

		for cnt in contours:
			cv2.drawContours(color_image, [cnt], -1, (0,255,0), 3)
			M = cv2.moments(cnt)
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			cv2.circle(color_image,(cx,cy), 5, (0,255,0), -1)

		self.image_pub.publish(self.bridge.cv2_to_imgmsg(color_image, "bgr8"))

	def ExtractPose(self, gray_data, depth_image):
		contours, hierarchy = cv2.findContours(gray_data, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]

		object_array = Detection2DArray()
		object_array.header.stamp = rospy.Time.now()
		object_array.header.frame_id = "camera_depth_frame"

		tray_list = []

		for cnt in contours:
			M = cv2.moments(cnt)
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])

			object_pose = Detection2D()
			object_pose.header.stamp = rospy.Time.now()
			object_pose.header.frame_id = "camera_depth_frame"

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

			# if x_meters < -0.117:
			# 	x_meters -= 0.025
			# elif x_meters > 0.117:
			# 	x_meters += 0.025
				
			# if y_meters < -0.067:
			# 	y_meters -= 0.015
			# elif y_meters > 0.067: 
			# 	y_meters += 0.015

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

		return object_array

	def depth_callback(self, depth_data):
		try:
			depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
		except CvBridgeError as e:
			print(e)

		if self.red_gray.any():

			red_object_array = self.ExtractPose(self.red_gray, depth_image)
			self.red_object_pub.publish(red_object_array)

		if self.blue_gray.any():

			blue_object_array = self.ExtractPose(self.blue_gray, depth_image)
			self.blue_object_pub.publish(blue_object_array)


if __name__ == '__main__':
	try:
		depth_camera = DepthCamera()
		rospy.spin()
	except KeyboardInterrupt:
		print('Shutting down the ROS Plate Detector Node')
		cv2.destroyAllWindows()


# def image_callback(msg):
# 	bridge = CvBridge()

# 	try:
# 		image = bridge.imgmsg_to_cv2(msg, "bgr8")
# 	except CvBridgeError as e:
# 		print(e)

# 	#Get camera intrinsic parameters
# 	camera_info = rospy.wait_for_message('/myur10/camera1/depth/camera_info', CameraInfo)
# 	fx = camera_info.K[0]
# 	fy = camera_info.K[4]
# 	pp_cx = camera_info.K[2]
# 	pp_cy = camera_info.K[5]

# 	# img_width = camera_info.width
# 	# img_height = camera_info.height

# 	# #Convert pixels to meters
# 	# sensor_width = img_width*fx
# 	# horizontal_fov = 2*np.arctan2(sensor_width, 2*fx)
# 	# x_conversion = 1/ (2*np.tan(horizontal_fov/2))

# 	# sensor_height = img_height*fy
# 	# vertical_fov = 2*np.arctan2(sensor_height, 2*fy)
# 	# y_conversion = 0.75*1/ (2*np.tan(vertical_fov/2))

# 	rows, cols = depth_image.shape
# 	u, v = np.meshgrid(np.arange(cols), np.arange(rows), indexing='xy')

# 	x = (u - pp_cx) * depth_image/fx
# 	y = (v - pp_cy) * depth_image/fy

# 	lower_red = np.array([0, 0, 60], dtype="uint8")
# 	upper_red = np.array([100, 50, 255], dtype="uint8")
# 	mask = cv2.inRange(image, lower_red, upper_red)
# 	output = cv2.bitwise_and(image, image, mask=mask)

# 	kernel = np.ones((3,3),np.uint8)
# 	erosion = cv2.erode(output, kernel, iterations = 4)
# 	dilate = cv2.dilate(erosion, kernel)

# 	gray = cv2.cvtColor(dilate, cv2.COLOR_BGR2GRAY)

# 	contours, hierarchy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# 	contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]

# 	for cnt in contours:
# 		cv2.drawContours(image, [cnt], -1, (255,0,0), 3)
# 		M = cv2.moments(cnt)
# 		cx = int(M['m10']/M['m00'])
# 		cy = int(M['m01']/M['m00'])
# 		cv2.circle(image,(cx,cy), 5, (255,0,255), -1)
# 		x_meters = x[cy][cx]
# 		y_meters = y[cy][cx]

# 		#Object Pose with respect to the center of image
# 		object_pose = Pose()
# 		object_pose.position.x = x_meters
# 		object_pose.position.y = y_meters
# 		object_pose.position.z = depth_image[cy][cx]/1000
		
# 		object_pose_quaternion = quaternion_from_euler(0, 0, 0)
# 		object_pose.orientation.x = object_pose_quaternion[0]
# 		object_pose.orientation.y = object_pose_quaternion[1]
# 		object_pose.orientation.z = object_pose_quaternion[2]
# 		object_pose.orientation.w = object_pose_quaternion[3]

# 		object_pub.publish(object_pose)


# 	cv2.imshow("Result", image)
# 	# cv2.imshow("Mask", output)
# 	cv2.waitKey(1)

# rospy.init_node('Red_Object_Detector')
# rospy.Subscriber('/myur10/camera1/depth/image_raw', Image, image_callback)
# object_pub = rospy.Publisher('red_object/position', Pose, queue_size=1)
# rospy.spin()
# cv2.destroyAllWindows()
