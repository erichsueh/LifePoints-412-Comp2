#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def joy_callback(msg):
    global started
    if(msg.buttons[2]==1):
        started = True
    elif(msg.buttons[1]==True):
        started = False

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("winright", 1)
        cv2.namedWindow("winleft", 1)
        cv2.namedWindow("window",1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, queue_size=1)
        self.twist = Twist()

    def image_callback(self, msg):
        global oldtime
        global started
        img = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        img = cv2.GaussianBlur(img,(5,5),0)
        #cv2.imshow("wondow",img)
        gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
        lower_white = numpy.array([numpy.max(gray)-30])
        upper_white = numpy.array([255])
        mask = cv2.inRange(gray, lower_white, upper_white)
        h, w, d = img.shape
        search_top = h/2
        search_bot = search_top + h/2
        mask[0:search_top,0:w]=0
        #mask[search_bot:h,0:w]=0
        #mask[0:220,0:w] = 0
        #mask = mask[search_top:]
        mask1 = mask[:, :w/4]
        mask2 = mask[:,w/4 *3:]
        #mask = numpy.asarray(mask)
        M2 = cv2.moments(mask2)
        M1 = cv2.moments(mask1)
        if (M1['m00'] > 0) and (M2['m00'] >0):
            cx1 = int(M1['m10']/M1['m00'])
            cx2 = int(M2['m10']/M2['m00']) + ((w/4) *3)
            cy1 = int(M1['m01']/M1['m00'])
            cy2 = int(M2['m01']/M2['m00'])
            cv2.circle(img, (cx1, cy1), 20, (0,0,255), -1)
            cv2.circle(img, (cx2, cy2), 20, (0,255,255), -1)
            err = (cx1 + cx2)/2 - w/2

            if started == True:
                self.twist.linear.x = .5
                self.twist.angular.z = -float(err) / 300
                self.cmd_vel_pub.publish(self.twist)
        elif(M1['m00']>0):
            #detecting left side turn left ish
            cx1 = int(M1['m10']/M1['m00'])
            cy1 = int(M1['m01']/M1['m00'])
            cv2.circle(img, (cx1, cy1), 20, (0,0,255), -1)
            err = cx1 - w/2
            if started == True:
                self.twist.linear.x = .5
                self.twist.angular.z = float(err) / 300
                self.cmd_vel_pub.publish(self.twist)
        elif(M2['m00']>0):
            cx2 = int(M2['m10']/M2['m00'])
            cy2 = int(M2['m01']/M2['m00'])
            cv2.circle(img, (cx2, cy2), 20, (0,0,255), -1)
            err = cx2 - w/2
            if started == True:
                self.twist.linear.x = .5
                self.twist.angular.z = float(err) / 300
                self.cmd_vel_pub.publish(self.twist)#detecting right side
            print("right")
        else:
            print("nothing")
            #no detection:
        cv2.imshow("winleft", mask1)
        cv2.imshow("winright",mask2)
        cv2.imshow("window",img)
        cv2.waitKey(3)

rospy.init_node('follower')
global oldtime
global started
started = False
joy_sub = rospy.Subscriber('joy',Joy, joy_callback)
oldtime = 0
follower = Follower()
rospy.spin()
