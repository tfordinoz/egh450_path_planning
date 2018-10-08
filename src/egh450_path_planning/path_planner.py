#!/usr/bin/env python

from math import *

import rospy
import tf2_ros
import tf_conversions

from nav_msgs.msg import Path
from std_msgs.msg import Time
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from breadcrumb.srv import RequestPath
from breadcrumb.srv import RequestPathRequest
from contrail_msgs.msg import WaypointList
from contrail_msgs.msg import DiscreteProgress
from contrail_msgs.srv import SetTracking, SetTrackingRequest


class PathPlanner():
	def __init__(self):
		
		# Wait for the breadcrumb interface to start up
		# then prepare a Service Client
		rospy.loginfo("[NAV] Waiting to connect with Breadcrumb")
		rospy.wait_for_service('~request_path')
		self.srvc_bc = rospy.ServiceProxy('~request_path', RequestPath)
		
		# Needs to be connected to contrail
		self.sub_wayp = rospy.Subscriber('~waypoints',WaypointList, self.request_path)
		self.pub_path = rospy.Publisher('~path', Path, queue_size=10, latch=True)
		
	def shutdown(self):
		# Unregister anything that needs it here
		pass
	def e_to_q(self,roll,pitch,yaw):
		cy = cos(yaw * 0.5);
		sy = sin(yaw * 0.5);
		cr = cos(roll * 0.5);
		sr = sin(roll * 0.5);
		cp = cos(pitch * 0.5);
		sp = sin(pitch * 0.5);

		w = cy * cr * cp + sy * sr * sp;
		x = cy * sr * cp - sy * cr * sp;
		y = cy * cr * sp + sy * sr * cp;
		z = sy * cr * cp - cy * sr * sp;

		return (w,x,y,z)

	def request_path(self,msgin):
		rospy.loginfo("[NAV] Requesting path from Breadcrumb")
		msg_out = Path()
		msg_out.header = msgin.header
	
		for i in range(len(msgin.waypoints) - 1):
			if msgin.waypoints[i].position.x == msgin.waypoints[i+1].position.x and msgin.waypoints[i].position.y == msgin.waypoints[i+1].position.y:
				ps = PoseStamped()
				ps.header = msgin.header
				ps.header.seq = i
				ps.pose.orientation.w = 1.0
				ps.pose.position.x = msgin.waypoints[i].position.x
				ps.pose.position.y = msgin.waypoints[i].position.y
				ps.pose.position.z = msgin.waypoints[i].position.z
				msg_out.poses.append(ps)
			else:
				
				#Request a path from breadcrumb
				req = RequestPathRequest()
			
				req.start = msgin.waypoints[i].position
				req.end = msgin.waypoints[i+1].position

				res = self.srvc_bc(req);
			
				if len(res.path.poses) > 0:
		
					#Insert the start pose
					ps = PoseStamped()
					ps.header = res.path_sparse.header
					ps.pose.position = req.start
					ps.pose.orientation.w = 1.0
					ps.pose.orientation.x = 0.0
					ps.pose.orientation.y = 0.0
					ps.pose.orientation.z = 0.0
					msg_out.poses.append(ps)
						
					
			
					# Insert the path recieved from breadcrumb
					for sp in res.path_sparse.poses:
						p = PoseStamped()	
						p.header = res.path_sparse.header
						p.pose.position = sp.position
						p.pose.orientation.w = 1.0
						p.pose.orientation.x = 0.0
						p.pose.orientation.y = 0.0
						p.pose.orientation.z = 0.0
						(w,x,y,z) = self.e_to_q(0.0,0.0,msgin.waypoints[i].yaw)
						p.pose.orientation.w = w
						p.pose.orientation.x = x
						p.pose.orientation.y = y
						p.pose.orientation.z = z
						msg_out.poses.append(p)

					#Insert the end pose
					pe = PoseStamped()
					pe.header = res.path_sparse.header
					pe.pose.position = req.end
					pe.pose.orientation.w = 1.0
					pe.pose.orientation.x = 0.0
					pe.pose.orientation.y = 0.0
					pe.pose.orientation.z = 0.0
					msg_out.poses.append(pe)
						
				else:
					rospy.logerr("[NAV] No path received, abandoning planning")
		self.pub_path.publish(msg_out)

class NavigationInterface():
	def __init__(self):
		# Needs to be connected to contrail & timestamp
		self.sub_trig = rospy.Subscriber('~imagery_trigger', Time, self.callback_trigger)
		self.sub_prog = rospy.Subscriber('~discrete_progress', DiscreteProgress, self.callback_progress)

		# Prepare diversion logic
		self.on_diversion = False
		self.pub_divert = rospy.Publisher('~pose', PoseStamped, queue_size=10)



		rospy.loginfo("[NAV] tf2_listener running.")

		# Create a listener to catch all TF2 messages
		self.tfBuffer = tf2_ros.Buffer()
		self.tfln = tf2_ros.TransformListener(self.tfBuffer)
		
		# Wait for the contrail interface to start up
		# then prepare a Service Client
		rospy.loginfo("[NAV] Waiting to connect with Contrail for Diversion")
		rospy.wait_for_service('~set_tracking')
		self.srv_track = rospy.ServiceProxy('~set_tracking', SetTracking)




	def shutdown(self):
		# Unregister anything that needs it here
		self.sub_trig.unregister()
		self.sub_prog.unregister()

	# Callback to handle an alert from image processing that the target is found
	def callback_trigger(self, msg_in):
		



		rospy.loginfo("[NAV] Got imagery trigger, setting diversion...")
		# We recieved a "found" timestamp
		# attempt to find the transformation
		try:

			# Lookup transform from "map" to "target" at time "msg_in.data",
			# and allow for 0.5 seconds to collected any additionally needed data
			t = self.tfBuffer.lookup_transform("map", "target", msg_in.data, rospy.Duration(0.5))

			# Dump information to screen
			rospy.loginfo("Found target at the following location in the world:")
			rospy.loginfo("[x: %0.2f; y: %0.2f; z: %0.2f]" % (t.transform.translation.x,
														  t.transform.translation.y,
														  t.transform.translation.z))

			msg_out = PoseStamped()
			msg_out.header.stamp = rospy.Time.now()
			msg_out.header.frame_id = "map"
			msg_out.pose.position.x = t.transform.translation.x
			msg_out.pose.position.y = t.transform.translation.y
			msg_out.pose.position.z = 0.5
			msg_out.pose.orientation.w = 1.0
			msg_out.pose.orientation.x = 0.0
			msg_out.pose.orientation.y = 0.0
			msg_out.pose.orientation.z = 0.0
			
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
				rospy.logwarn(e)


		# Publish the diversion and set the flag for the progress callback
		self.pub_divert.publish(msg_out)
		self.on_diversion = True

	# Callback to handle progress updates from Contrail
	def callback_progress(self, msg_in):
		#If we were on a diversion, and have reached the drop point
		if self.on_diversion and (msg_in.progress == 1.0):
			rospy.loginfo("[NAV] Reached drop point, requesting path continue...")

			# Time and payload stuff here
			rospy.sleep(rospy.Duration(5))

			# Disable further tracking changes (unless re-triggered)
			self.on_diversion = False

			# Send a request to contrail to resume path tracking
			req = SetTrackingRequest()
			req.tracking = req.TRACKING_PATH
			res = self.srv_track(req)

			if res.success:
				rospy.loginfo("[NAV] Diversion complete!")
			else:
				rospy.logerr("[NAV] Could not return to path tracking!")
