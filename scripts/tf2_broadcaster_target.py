#!/usr/bin/env python

import math

import rospy
import tf2_ros
from std_msgs.msg import Time
from geometry_msgs.msg import TransformStamped

# Global Variables
tfbr = None
pub_found = None

camera_name = "camera"
target_name = "target"

def send_tf_target():
	# Generate our "found" timestamp
	time_found = rospy.Time.now()

	# Create a transform arbitrarily in the
	# camera frame
	t = TransformStamped()
	t.header.stamp = time_found
	t.header.frame_id = camera_name
	t.child_frame_id = target_name

	t.transform.translation.x = -0.4
	t.transform.translation.y = 0.2
	t.transform.translation.z = 1.5
	t.transform.rotation.x = 0.0
	t.transform.rotation.y = 0.0
	t.transform.rotation.z = 0.0
	t.transform.rotation.w = 1.0

	# Send the transformation to TF
	# and "found" timestamp to localiser
	tfbr.sendTransform(t)
	pub_found.publish(time_found)

if __name__ == '__main__':
	rospy.init_node('tf2_broadcaster_target')
	rospy.loginfo("tf2_broadcaster_target sending target found...")

	# Setup tf2 broadcaster and timestamp publisher
	tfbr = tf2_ros.TransformBroadcaster()
	pub_found = rospy.Publisher('/emulated_uav/target_found', Time, queue_size=10)

	# Give the nodes a few seconds to configure
	rospy.sleep(rospy.Duration(2))

	# Send out our target messages
	send_tf_target()

	# Give the nodes a few seconds to transmit data
	# then we can exit
	rospy.sleep(rospy.Duration(2))
	rospy.loginfo("tf2_broadcaster_target sent TF and timestamp")
