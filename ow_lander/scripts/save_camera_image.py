#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess  # Import the subprocess module to run other ROS nodes
import os

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

    # Range of pan values from -3.2 to 3.2
    pan_values = [x / 10.0 for x in range(-32, 32+1)]

    # Range of tilt values from -3.2 to 3.2
    tilt_values = [x / 10.0 for x in range(-15, 15+1)]

    package_name = "ow_lander"
    # init 
    pan0 = -3.2
    pan_args = [f"{pan0:.1f}"]
    run_other_ros_nodes("pan.py", package_name, pan_args)
    print("init pan", pan0)
    # rospy.sleep(5)
    tilt0 = -1.5
    tilt_args = [f"{tilt0:.1f}"]
    run_other_ros_nodes("tilt.py", package_name, tilt_args)
    print("init tilt", tilt0)
    # rospy.sleep(5)


    for pan in pan_values:
        # Step 1: Run script_1.py with the specified pan value
        script_name = "pan.py"
        pan_args = [f"{pan:.1f}"]
        run_other_ros_nodes(script_name, package_name, pan_args)
        print("pan", pan_args)
        # rospy.sleep(0.5)

        for tilt in tilt_values:
            # Step 2: Run tilt.py with the specified tilt value
            script_name = "tilt.py"
            tilt_args = [f"{tilt:.1f}"]
            run_other_ros_nodes(script_name, package_name, tilt_args)
            print("tilt", tilt_args)
            #if(tilt == -1.5):
            #    rospy.sleep(5)
            #else:
            #    rospy.sleep(0.5)

            # Step 3: Run script_2.py to capture the camera view
            script_name = "camera_capture.py"
            run_other_ros_nodes(script_name, package_name, [])
            #rospy.sleep(1)
            print("camera_capture")

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
