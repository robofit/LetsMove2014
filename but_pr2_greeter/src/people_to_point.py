#!/usr/bin/python

import roslib
roslib.load_manifest('but_pr2_greeter')
import rospy

from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import PointStamped

from math import pow, sqrt

class PeopleDet2Point:
    
    def __init__(self, debug = False):
    
        self._debug = debug
        
        self._point_pub = rospy.Publisher('nearest_face', PointStamped)
        self._det_sub = rospy.Subscriber('/face_detector/people_tracker_measurements_array', PositionMeasurementArray, self.face_cb)
     
    def publish(self, msg):
        
        pt = PointStamped()
        
        pt.header = msg.header
        pt.point = msg.pos   
        
        self._point_pub.publish(pt)
        
    def face_cb(self,msg):
        
        dist = None
        idx = None
        
        for iidx in range(0,len(msg.people)):
            
            d = sqrt(pow(msg.people[iidx].pos.x,2) + pow(msg.people[iidx].pos.y,2) + pow(msg.people[iidx].pos.z,2))
            
            if dist is None:
                
                dist = d
                idx = iidx
                continue
                
            if dist < d:
                
                dist = d
                idx = iidx
                
        if idx is not None:
            
            self.publish(msg.people[idx])
        
        
        
if __name__ == '__main__':
    
    rospy.init_node('but_pr2_det_converter')
    rospy.loginfo("PR2 detection converter")
    
    bpg = PeopleDet2Point(debug=True)
    
    rospy.spin()