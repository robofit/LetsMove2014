#!/usr/bin/python

import roslib
roslib.load_manifest('but_pr2_greeter')
import rospy

import random

from tf import TransformListener
from geometry_msgs.msg import PointStamped

class FakeFaceDetector:
    
    def __init__(self, debug=False, cam_frame_id = "/head_mount_kinect_rgb_optical_frame", robot_frame_id = "/base_link"):
        
        self.tfl = TransformListener()
        
        rospy.sleep(2)
        
        self.debug = debug
        
        self._point_pub = rospy.Publisher('nearest_face', PointStamped)
        
        self.cam_frame_id = cam_frame_id
        self.robot_frame_id = robot_frame_id
        
        self.det_duration = rospy.Duration(20)
        self.det_last = rospy.Time(0)
        
        self.pt = PointStamped()
        
        self._timer = rospy.Timer(rospy.Duration(0.1), self.timer)
        
    def timer(self,event):        
        
        now = rospy.Time.now()
        self.pt.header.stamp = now - rospy.Duration(0.5)
        
        if (self.det_last + self.det_duration < now):
        
            self.det_last = now
        
            self.pt.header.frame_id = self.robot_frame_id
            
            self.pt.point.x = random.uniform(1.5, 3.5)
            self.pt.point.y = random.uniform(-0.5,0.5)
            self.pt.point.z = random.uniform(1.3, 2.0)
            
            if self.debug:
            
                rospy.loginfo("Fake face at: [" + str(self.pt.point.x) + ", " + str(self.pt.point.y) + ", " + str(self.pt.point.z) + "] (" + self.robot_frame_id + ")")
                
                
        
        pt = self.pt
        
        pt.point.x = random.uniform(pt.point.x - 0.05, pt.point.x + 0.05)
        pt.point.y = random.uniform(pt.point.y - 0.05, pt.point.y + 0.05)
        pt.point.z = random.uniform(pt.point.z - 0.05, pt.point.z + 0.05)
        
        # TODO "simulate" movement?
        
        try:
            
            pt = self.tfl.transformPoint(self.cam_frame_id, pt)
            
        except:
            
            rospy.logerr("Transform error")
            self.det_last = rospy.Time(0)
            return
        
        self._point_pub.publish(pt)
        
        
if __name__ == '__main__':
    
    rospy.init_node('but_pr2_fake_face_det')
    rospy.loginfo("Fake face detector")
    
    ffd = FakeFaceDetector(debug=True)
    
    rospy.spin()