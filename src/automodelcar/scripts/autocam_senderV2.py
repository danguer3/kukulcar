import rospy
import numpy
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge

def talker():
        pub = rospy.Publisher('cam', Image)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        cap = cv.VideoCapture(1)
        if not cap.isOpened():
                print("Cannot open camera")
                exit()
        while not rospy.is_shutdown():
                ret, img = cap.read()
                if not ret:
                        print("Can't receive frame (stream end?). Exiting ...")
                        break
                scale_percent = 5 # percent of original size
                width = int(img.shape[1] * scale_percent / 100)
                height = int(img.shape[0] * scale_percent / 100)
                dim = (width, height)
                resized = cv.resize(img, dim, interpolation = cv.INTER_AREA)
                bridge = CvBridge()
                image_message = bridge.cv2_to_imgmsg(resized, encoding="bgr8")
                rospy.loginfo(image_message)
                pub.publish(image_message)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
