#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np

class egg_iden:
    def __init__(self):
        self.webcam = cv.VideoCapture(2)
        width = 1920
        height = 1080
        self.webcam.set(cv.CAP_PROP_FRAME_WIDTH, width)
        self.webcam.set(cv.CAP_PROP_FRAME_HEIGHT, height)

        self.dt = 0.1

        rospy.init_node('egg_iden')

        #self.s1 = rospy.Subscriber('/video_source/raw',Image,self.img_callback) #Recibe el video
        
        self.red_mask = rospy.Publisher('/red_mask', Image, queue_size=10)
        self.yel_mask = rospy.Publisher('/yel_mask', Image, queue_size=10)
        self.mag_mask = rospy.Publisher('/mag_mask', Image, queue_size=10)
        self.gre_mask = rospy.Publisher('/gre_mask', Image, queue_size=10)
        self.blu_mask = rospy.Publisher('/blu_mask', Image, queue_size=10)
        self.ora_mask = rospy.Publisher('/ora_mask', Image, queue_size=10)
        self.camera = rospy.Publisher('/camera', Image, queue_size=10)

        self.den_red = rospy.Publisher('/den_red', Float32, queue_size=10)
        self.den_yel = rospy.Publisher('/den_yel', Float32, queue_size=10)
        self.den_mag = rospy.Publisher('/den_mag', Float32, queue_size=10)
        self.den_ora = rospy.Publisher('/den_ora', Float32, queue_size=10)
        self.den_blu = rospy.Publisher('/den_blu', Float32, queue_size=10)
        self.den_gre = rospy.Publisher('/den_gre', Float32, queue_size=10)

        self.frame = None
        self.bridge = CvBridge()
        self.t1 = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)
        self.rate = rospy.Rate(10)

    def img_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rospy.loginfo('Connected to camera')
        except:
            rospy.loginfo('Failed to convert image to CV2 image.')
            
    def timer_callback(self, t):
        try:
            self.egg_identification()
        except Exception as e:
            print(e)
            pass

    def stop(self):
        print('Killing egg_iden')

    def egg_identification(self):
        _, frame = self.webcam.read()

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        
        low_red = np.array([170, 140, 0], np.uint8)
        upp_red = np.array([195, 195, 255], np.uint8)
        low_yel = np.array([13, 30, 0], np.uint8)
        upp_yel = np.array([55, 95, 255], np.uint8)
        low_mag = np.array([110, 50, 0], np.uint8)
        upp_mag = np.array([135, 125, 255], np.uint8)
        low_ora = np.array([0, 80, 0], np.uint8)
        upp_ora = np.array([75, 156, 255], np.uint8)
        low_blu = np.array([100, 150, 0], np.uint8)
        upp_blu = np.array([110, 215, 255], np.uint8)
        low_gre = np.array([60, 40, 0], np.uint8)
        upp_gre = np.array([99, 165, 255], np.uint8)

        low = [low_red, low_yel, low_mag, low_ora, low_blu, low_gre]
        upp = [upp_red, upp_yel, upp_mag, upp_ora, upp_blu, upp_gre]

        # CAMBIAR POR EL TOPICO PUBLICADO (POR DEFINIR)

        # if cv.waitKey(1) == ord('d'):
        #     i += 1
        #     if i == 6:
        #         i = 0

        mask = cv.inRange(hsv, low[0], upp[0])

        kernel = np.ones((2,2),np.uint8)

        mask1 = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        mask1 = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)

        inv_mask = cv.bitwise_not(mask1)

        crop_frame1 = inv_mask[200:500,450:770] #Cambiar por cada frame
        crop_frame2 = inv_mask[480:800,450:727]
        crop_frame3 = inv_mask[800:1080,450:770]
        crop_frame4 = inv_mask[200:480,1050:1380]
        crop_frame5 = inv_mask[480:800,1050:1380]
        crop_frame6 = inv_mask[800:1080,1050:1380]

        crop_frame_1 = cv.bitwise_not(crop_frame1)
        crop_frame_2 = cv.bitwise_not(crop_frame2)
        crop_frame_3 = cv.bitwise_not(crop_frame3)
        crop_frame_4 = cv.bitwise_not(crop_frame4)
        crop_frame_5 = cv.bitwise_not(crop_frame5)
        crop_frame_6 = cv.bitwise_not(crop_frame6)

        density1 = np.sum(crop_frame_1)/(crop_frame_1.shape[1]*crop_frame_1.shape[0]*255)
        density2 = np.sum(crop_frame_2)/(crop_frame_2.shape[1]*crop_frame_2.shape[0]*255)
        density3 = np.sum(crop_frame_3)/(crop_frame_3.shape[1]*crop_frame_3.shape[0]*255)
        density4 = np.sum(crop_frame_4)/(crop_frame_4.shape[1]*crop_frame_4.shape[0]*255)
        density5 = np.sum(crop_frame_5)/(crop_frame_5.shape[1]*crop_frame_5.shape[0]*255)
        density6 = np.sum(crop_frame_6)/(crop_frame_6.shape[1]*crop_frame_6.shape[0]*255)
        
        self.red_mask.publish(self.bridge.cv2_to_imgmsg(crop_frame1, "mono8"))
        self.yel_mask.publish(self.bridge.cv2_to_imgmsg(crop_frame2, "mono8"))
        self.mag_mask.publish(self.bridge.cv2_to_imgmsg(crop_frame3, "mono8"))
        self.ora_mask.publish(self.bridge.cv2_to_imgmsg(crop_frame4, "mono8"))
        self.blu_mask.publish(self.bridge.cv2_to_imgmsg(crop_frame5, "mono8"))
        self.gre_mask.publish(self.bridge.cv2_to_imgmsg(crop_frame6, "mono8"))

        self.den_red.publish(density1)  
        self.den_yel.publish(density2)
        self.den_mag.publish(density3)
        self.den_ora.publish(density4)
        self.den_blu.publish(density5)
        self.den_gre.publish(density6)  

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    egg = egg_iden()

    try:
        egg.run()
    except:
        pass
