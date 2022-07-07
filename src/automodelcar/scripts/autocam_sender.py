#!/usr/bin/env python
import rospy
import numpy
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge

def talker():
	pub = rospy.Publisher('/camera/rgb/raw', Image,queue_size=10)
	rospy.init_node('camera_sender', anonymous=True)
	rate = rospy.Rate(10)
	cap = cv.VideoCapture('Videod.avi')
	#cap.set(cv.CAP_PROP_FRAME_WIDTH,640);
	#cap.set(cv.CAP_PROP_FRAME_HEIGHT,480);
	bridge = CvBridge()
	if not cap.isOpened():
		print("Cannot open camera")
		exit()
	while not rospy.is_shutdown():
		ret, frame = cap.read()
		if not ret:
			print("Can't receive frame (stream end?). Exiting ...")
			break
		dim = (640,480)
		frame = cv.resize(frame,dim)
		print(frame)
		cv.imshow('Cam',frame)
		print(frame.shape)
		image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
		#rospy.loginfo(image_message)
		pub.publish(image_message)
		rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

