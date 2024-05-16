#!/usr/bin/env python

# This node developed in Python allows the localization of objects (xyz coordinates) viewed by a camera,
# using an RGB camera and a depth image. 
# By filtering the color of an RGB image to HSV, the centroid of each detected object is obtained, and,
# knowing the depth (camera-object distance) and the intrinsic parameters of the camera,
# it is possible to calculate the XYZ coordinates of the objects with respect to the camera. 

# A template is provided to facilitate the development of this node and "TODO" comments indicate where necessary functions should be added.

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from math import sqrt, pow

# Declare global variables (publishers and flags for detected objects)
filtered_obj_pub = None
filtered_blue_pub = None
pose_array_pub = None
obt_detec = False
robot_detec = False
depth_image = None  # Added global variable for depth image
print_count = 10

def filter_img_objects(color_image, lower, upper):
    # Convert the image from RGB to HSV for filtering
    hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

    # Filter the image and obtain the mask
    mask = cv2.inRange(hsv_image, lower, upper)
    filtered_image = cv2.bitwise_and(color_image, color_image, mask=mask)

    # Find the contours of the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    centroid = None
    detection = False

    # Calculate the centroid of the mask
    if contours:
        # Find the contour with the largest area
        largest_contour = max(contours, key=cv2.contourArea)
        # Calculate the moments of the contour
        M = cv2.moments(largest_contour)
        # Calculate the centroid coordinates
        if M["m00"] != 0:
            centroid = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            detection = True

    return filtered_image, centroid, detection

def img_xyz(centroid, depth_image, camera_matrix):
    # Extract the intrinsic calibration parameters of the camera
    fx, fy = camera_matrix[0], camera_matrix[4]
    cx, cy = camera_matrix[2], camera_matrix[5]

    # Centroid coordinates
    px, py = centroid

    # Depth (Z coordinate)
    z = depth_image[py, px] / 1000.0  # Convert depth from mm to meters

    # Calculate X and Y coordinates
    x = (px - cx) * z / fx
    y = (py - cy) * z / fy

    return x, y, z

def camera_info_callback(camera_info_msg):
    global centroide_obj, centroide_blue, depth_image, depth_blue, pose_array_pub, obt_detec, robot_detec, print_count

    # Initialize XYZ coordinates of the objects
    x_obj, y_obj, z_obj = 0, 0, -1
    x_blue, y_blue, z_blue = 0, 0, -1

    if obt_detec and depth_image is not None:
        x_obj, y_obj, z_obj = img_xyz(centroide_obj, depth_image, camera_info_msg.K)

    if robot_detec and depth_image is not None:
        x_blue, y_blue, z_blue = img_xyz(centroide_blue, depth_image, camera_info_msg.K)

    # Invert the Y axis to match global axes
    y_obj, y_blue = -y_obj, -y_blue

    # Print the results    
    print_count -= 1
    if print_count == 0:
       print("XYZ object: {:.3f}, {:.3f}, {:.3f}".format(x_obj, y_obj, z_obj))
       print("XYZ robot : {:.3f}, {:.3f}, {:.3f}".format(x_blue, y_blue, z_blue))
       print_count = 10

    # Create a PoseArray message
    pose_array_msg = PoseArray()
    pose_array_msg.header.stamp = rospy.Time.now()
    pose_array_msg.header.frame_id = "camera_link"

    # Object pose
    pose_obj = Pose()
    pose_obj.position.x = x_obj
    pose_obj.position.y = y_obj
    pose_obj.position.z = z_obj
    pose_obj.orientation.w = 1.0

    # Robot pose
    pose_blue = Pose()
    pose_blue.position.x = x_blue
    pose_blue.position.y = y_blue
    pose_blue.position.z = z_blue
    pose_blue.orientation.w = 1.0

    # Add the poses to the PoseArray
    pose_array_msg.poses.append(pose_obj)
    pose_array_msg.poses.append(pose_blue)

    # Publish the PoseArray
    pose_array_pub.publish(pose_array_msg)

def color_image_callback(color_image_msg):
    global filtered_obj_pub, filtered_blue_pub, centroide_obj, centroide_blue, obt_detec, robot_detec

    # Convert the image message to an OpenCV image
    bridge = CvBridge()
    color_image = bridge.imgmsg_to_cv2(color_image_msg, desired_encoding="bgr8")

    # Filter the blue robot
    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([120, 255, 255])
    filtered_blue, centroide_blue, robot_detec = filter_img_objects(color_image, lower_blue, upper_blue)

    # Publish the filtered image of the robot
    filtered_image_msg = bridge.cv2_to_imgmsg(filtered_blue, encoding="bgr8")
    filtered_blue_pub.publish(filtered_image_msg)

    # Filter the red object (mask limits in HSV)
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    filtered_obj, centroide_obj, obt_detec = filter_img_objects(color_image, lower_red, upper_red)

    # Publish the filtered image of the object
    filtered_image_msg = bridge.cv2_to_imgmsg(filtered_obj, encoding="bgr8")
    filtered_obj_pub.publish(filtered_image_msg)

def depth_image_callback(depth_image_msg):
    global depth_image

    # Convert the depth image message to an OpenCV image
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding="passthrough")

def ros_node():
    rospy.init_node('Object_localization', anonymous=True)

    # Subscribe to topics:
    # - RGB image from the camera
    # - Depth image from the camera
    # - Camera info with intrinsic parameters

    color_image_sub = rospy.Subscriber('/camera/color/image_raw', Image, color_image_callback)
    depth_image_sub = rospy.Subscriber('/camera/depth/image_raw', Image, depth_image_callback)
    camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, camera_info_callback)

    global filtered_obj_pub, filtered_blue_pub, pose_array_pub

    # Publish results (filtered images and poses of the objects)
    filtered_obj_pub = rospy.Publisher('/filtered_image/object', Image, queue_size=1)
    filtered_blue_pub = rospy.Publisher('/filtered_image/robot', Image, queue_size=1)
    pose_array_pub = rospy.Publisher('/pose_array', PoseArray, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        ros_node()
    except rospy.ROSInterruptException:
        pass
        