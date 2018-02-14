#!/usr/bin/env python
import sys
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class Racebot:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)

        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                           Image, self.image_callback)
        
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)

        self.twist = Twist()

        try:
            self.savedMatrix = numpy.load("savedMatrix.npy")
            print("Loaded")
        except IOError:
            print "No file exists, train first"
            sys.exit(0)

        self.begin = False            

    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        image = cv2.GaussianBlur(image,(5,5),0)
        cv2.imshow("window", image)

        if self.begin:
    
            gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    
            lower_white = numpy.array([numpy.max(gray)-30])
            upper_white = numpy.array([255])
    
            mask = cv2.inRange(gray, lower_white, upper_white)
    
   
            h,w,d = image.shape
            search_top = 220

            mask = mask[search_top:]
            mask = numpy.asarray(mask)

            filtered = numpy.zeros(len(mask)*len(mask[0])/4)

            k = 0

            for i in range(0,len(mask),5):
                for j in range(0,len(mask[0]),5):
                    filtered[k] = (numpy.sum(mask[i:i+5, j:j+5]))

                    filtered[k] = filtered[k]/(25*255)
                    k = k + 1

            vels = numpy.dot(filtered,self.savedMatrix)
            self.twist.linear.x = vels[0]
            self.twist.angular.z = vels[1]
            print vels
            self.cmd_vel_pub.publish(self.twist)

    def joy_callback(self, msg):
        if msg.buttons[2] == 1:
            self.begin = True

        if msg.buttons[1] == 1:
            self.begin = False





rospy.init_node('racebot')
racebot = Racebot()
rospy.spin()