#!/usr/bin/env python3
import rospy

import tf2_ros
import tf_conversions

def main():
    rospy.init_node('camera_tf_listener')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10.0)  # Adjust the rate based on your requirements

    values_printed = False

    while not rospy.is_shutdown() and not values_printed:
        try:
            # Get the transform from /base_link to /StereoCameraCenter_optical_frame
            transform_stamped = tf_buffer.lookup_transform('base_link', 'StereoCameraCenter_optical_frame', rospy.Time(0))

            # Extract translation and rotation
            translation = [transform_stamped.transform.translation.x,
                           transform_stamped.transform.translation.y,
                           transform_stamped.transform.translation.z]

            rotation = tf_conversions.transformations.euler_from_quaternion([
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z,
                transform_stamped.transform.rotation.w
            ])

            print("Translation:", translation)
            print("Rotation (RPY):", rotation)

            values_printed = True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to get TF transform")

        rate.sleep()

if __name__ == '__main__':
    main()

