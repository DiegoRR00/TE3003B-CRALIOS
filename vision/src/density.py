#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Int8
import numpy as np

class frank:
    def __init__(self):
        self.dt = 0.1
        self.density = None

        rospy.init_node('detector_frank')

        self.s1 = rospy.Subscriber('/den_red',Float32,self.den_red_callback) #Recibe densidad
        self.s2 = rospy.Subscriber('/den_yel',Float32,self.den_yel_callback) #Recibe densidad
        self.s3 = rospy.Subscriber('/den_mag',Float32,self.den_mag_callback) #Recibe densidad
        self.s4 = rospy.Subscriber('/den_ora',Float32,self.den_ora_callback) #Recibe densidad
        self.s5 = rospy.Subscriber('/den_blu',Float32,self.den_blu_callback) #Recibe densidad
        self.s6 = rospy.Subscriber('/den_gre',Float32,self.den_gre_callback) #Recibe densidad

        self.frame_detected = rospy.Publisher('/frame_detected', Int8, queue_size=10)
        self.flag_assembled = rospy.Subscriber('flag_assembled', Int8, queue_size=10)
        self.flag = rospy.Publisher('flag_assembled', Int8, queue_size=10)

        self.t1 = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)
        self.rate = rospy.Rate(10)

    def den_red_callback(self, msg):
        try:
            self.den_Red = msg.data
        except:
            rospy.loginfo('Failed to obtain image density.')
    
    def den_yel_callback(self, msg):
        try:
            self.den_Yel = msg.data
        except:
            rospy.loginfo('Failed to obtain image density.')

    def den_mag_callback(self, msg):
        try:
            self.den_Mag = msg.data
        except:
            rospy.loginfo('Failed to obtain image density.')

    def den_ora_callback(self, msg):
        try:
            self.den_Ora = msg.data
        except:
            rospy.loginfo('Failed to obtain image density.')

    def den_blu_callback(self, msg):
        try:
            self.den_Blu = msg.data
        except:
            rospy.loginfo('Failed to obtain image density.')

    def den_gre_callback(self, msg):
        try:
            self.den_Gre = msg.data
        except:
            rospy.loginfo('Failed to obtain image density.')

    def timer_callback(self, t):
        try:
            self.density_eval()
        except:
            pass

    def stop(self):
        print('Killing Frank')

    def density_eval(self):
        flag = Bool()
        flag.data = False

        # density = [self.den_Red, self.den_Yel, self.den_Mag, self.den_Ora, self.den_Blu, self.den_Gre]
        density = [self.den_Ora, self.den_Blu, self.den_Gre, self.den_Red, self.den_Yel, self.den_Mag]

        if np.max(density) >= 0.2:
            msg = density.index(max(density))
            self.frame_detected.publish(msg + 1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    frank = frank()

    try:
        frank.run()
    except:
        pass