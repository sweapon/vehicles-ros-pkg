#!/usr/bin/env python
'''
Created on Feb 04, 2013

@author: dnad
'''
import rospy;
import math;
import random;
import numpy
from sensor_msgs.msg import Joy  
from auv_msgs.msg import BodyForceReq
from std_msgs.msg import Int16MultiArray
     
class Buddymore:
    def __init__(self):
        
        self.joy= rospy.Subscriber('joy', Joy, self.onJoy);
        
        self.tauOut = rospy.Publisher('tauOut', BodyForceReq)
        self.pwmOut= rospy.Publisher('pwm_out', Int16MultiArray)       
        
    def onJoy(self,data):
        tauX = data.axes[1];
        tauY = -data.axes[0];
        tauZ = -data.axes[3];
        tauN = -data.axes[2];
        tmax = 0.7*255;
                
        t1 = -(tauX+tauN)/2;
        t2 = (tauX-tauN)/2;
        
        t3 = tauZ - tauY;
        t4 = -(tauZ+tauY);          
              
        out = Int16MultiArray()
        out.data = tmax*numpy.array([t1,t2,t3,t4])
        self.pwmOut.publish(out);
                                        
if __name__ == "__main__":
    rospy.init_node("buddymore");
    mark = Buddymore(); 
    
    while not rospy.is_shutdown():
        rospy.spin();        
        
        
