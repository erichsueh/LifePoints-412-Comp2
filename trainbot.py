#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy, scipy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class Trainbot:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)

        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel_mux/input/teleop', Twist, self.cmd_vel_callback)
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

        try:
            self.matrixA = numpy.load("matrixA.npy")
        except IOError:
            self.matrixA = numpy.asarray([])
            print "Did not load"
            print len(self.matrixA)
        try:
            self.matrixB = numpy.load("matrixB.npy")
        except:
            self.matrixB = numpy.asarray([])

        print "Loaded"

        self.lastvel = numpy.zeros(2)

        self.begin = False

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        image = cv2.GaussianBlur(image,(5,5),0)

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


            if self.matrixA.size == 0:
                self.matrixA = filtered
                self.matrixB = self.lastvel
            else:
                self.matrixA = numpy.vstack([self.matrixA,filtered])
                self.matrixB = numpy.vstack([self.matrixB, self.lastvel])

            print len(self.matrixA)

    def cmd_vel_callback(self, msg):
        self.lastvel[0] = msg.linear.x
        self.lastvel[1] = msg.angular.z

    def joy_callback(self, msg):
        if msg.buttons[2] == 1:
            self.begin = True

        if msg.buttons[1] == 1:
            self.begin = False
            answer = numpy.linalg.lstsq(self.matrixA,self.matrixB)[0]
            numpy.save("savedMatrix.npy",answer)
            numpy.save("matrixA.npy", self.matrixA)
            numpy.save("matrixB.npy", self.matrixB)
            print "Saved"

    

rospy.init_node('trainbot')
trainbot = Trainbot()
rospy.spin()