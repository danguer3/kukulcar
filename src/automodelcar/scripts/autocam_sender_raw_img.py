import rospy
import numpy
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge

def talker():
	pub = rospy.Publisher('cam', Image, queue_size=10)
	rospy.init_node('raw_image', anonymous=True)
	rate = rospy.Rate(60)
	cap = cv.VideoCapture(0)
	if not cap.isOpened():
		print("Cannot open camera")
		exit()
	while not rospy.is_shutdown():
		ret, frame = cap.read()
		if not ret:
			print("Can't receive frame (stream end?). Exiting ...")
			break
		#print(frame)
		bridge = CvBridge()
		image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
		rospy.loginfo(image_message)
		pub.publish(image_message)
		rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

