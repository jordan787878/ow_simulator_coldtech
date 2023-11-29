#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess  # Import the subprocess module to run other ROS nodes
import os

# image_directory: path to save the image
# in line-64, I hard code the right camera to be subscribed, you can add left camera as well.

def run_other_ros_nodes(script_name, package_name, args):
    try:
        full_command = ["rosrun", package_name, script_name] + args
        subprocess.check_call(full_command)  # Run the ROS nodes
    except subprocess.CalledProcessError as e:
        rospy.logerr("Error running ROS node: %s", e)
        return

def image_callback(msg, image_name):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as e:
        rospy.logerr(e)
        return

    # Define the directory where the image will be saved
    image_directory = "/home/coldtech/Pictures/camera_images/"

    # Generate the image filename
    image_filename = os.path.join(image_directory, image_name) 

    # Save the image
    cv2.imwrite(image_filename, cv_image)
    rospy.loginfo("Saved image to %s", image_filename)

def main():
    rospy.init_node('image_capture_and_pan')

    package_name = "ow_lander"

    # pan value
    pan0 = 1.5
    pan_args = [f"{pan0:.1f}"]
    run_other_ros_nodes("pan.py", package_name, pan_args)
    print("set pan", pan0)

    # tilt value
    tilt0 = 0.8
    tilt_args = [f"{tilt0:.1f}"]
    run_other_ros_nodes("tilt.py", package_name, tilt_args)
    print("set tilt", tilt0)
    rospy.sleep(1)

    # save image to image directory
    script_name = "camera_capture.py"
    run_other_ros_nodes(script_name, package_name, [])
    print("camera_capture")
    rospy.sleep(2)

    image_name = f"image_pan{pan_args}_tilt{tilt_args}.jpg"
    # Step 3: Subscribe to the image topic and save the image
    image_sub = rospy.Subscriber('/StereoCamera/right/image_raw', Image, 
                                image_callback, callback_args=image_name)

    # Wait for a short time to ensure image capture
    rospy.sleep(1)

    # Unsubscribe from the image topic
    image_sub.unregister()

    rospy.signal_shutdown("Image capture and pan complete")


if __name__ == '__main__':
    main()
