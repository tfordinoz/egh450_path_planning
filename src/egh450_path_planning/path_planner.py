#!/usr/bin/env python

from math import *

import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from breadcrumb.srv import RequestPath
from breadcrumb.srv import RequestPathRequest
from contrail_msgs.msg import WaypointList

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
					ps.header = res.path.header
					ps.pose.position = req.start
					ps.pose.orientation.w = 1.0
					ps.pose.orientation.x = 0.0
					ps.pose.orientation.y = 0.0
					ps.pose.orientation.z = 0.0
					msg_out.poses.append(ps)
						
					
			
					# Insert the path recieved from breadcrumb
					for sp in res.path.poses:
						p = PoseStamped()	
						p.header = res.path.header
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
					pe.header = res.path.header
					pe.pose.position = req.end
					pe.pose.orientation.w = 1.0
					pe.pose.orientation.x = 0.0
					pe.pose.orientation.y = 0.0
					pe.pose.orientation.z = 0.0
					msg_out.poses.append(pe)
						
				else:
					rospy.logerr("[NAV] No path received, abandoning planning")
		self.pub_path.publish(msg_out)





