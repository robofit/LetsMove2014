#!/usr/bin/python

import roslib
roslib.load_manifest('but_pr2_greeter')
import rospy

from geometry_msgs.msg import PointStamped
from tf import TransformListener

class PR2Greeter:
    
    def __init__(self,debug = False, robot_frame = "odom_combined"):
        
        self._tf = TransformListener()
        
        self._debug = debug
        self._robot_frame = robot_frame
        
        self._point_sub = rospy.Subscriber('nearest_face', PointStamped, self.face_cb)
        
        
    def face_cb(self,point):
        
        rospy.loginfo("Point received")
        
        if not self._tf.frameExists(point.header.frame_id):
            
            rospy.logerr("Frame " + point.header.frame_id + " doesn't exist!")
            return
        
        if not self._tf.frameExists(self._robot_frame):
            
            rospy.logerr("Frame " + self._robot_frame + " doesn't exist!")
            return
        
        # transform point
        
        # point PR2's head there (http://wiki.ros.org/pr2_controllers/Tutorials/Moving%20the%20Head)
        
        # wave the hand
        
        
if __name__ == '__main__':
    
    rospy.init_node('but_pr2_greeter')
    rospy.loginfo("PR2 Greeter")
    
    bpg = PR2Greeter(debug=False)
    
    rospy.spin()