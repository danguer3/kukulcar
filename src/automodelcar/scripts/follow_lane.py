#!/usr/bin/env python
import rospy
import numpy
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32
steering = 0
def callback(msg):
    global steering
    angle_l = msg.data[0]
    angle_r = msg.data[1]
    angle_d = 0.4
    e_angle_l = angle_d - angle_l
    e_angle_r = angle_d - angle_r
    e_angle = (e_angle_l + e_angle_l)/2
    steering = -e_angle*180/numpy.pi
    if abs(steering) > 30.0 :
	steering = 30.0

def lane_follower():
    global steering
    pub_speed = rospy.Publisher('speed', Float32, queue_size=1)
    pub_steering = rospy.Publisher('steering', Float32, queue_size=1)
    rospy.init_node('lane_follower', anonymous=True)
    rospy.Subscriber("/steering_angle", Float64MultiArray, callback)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub_speed.publish(Float32(data = 0.00005))
        pub_steering.publish(Float32(data = steering))
        rate.sleep()

if __name__ == '__main__':
    try:
        lane_follower()
    except rospy.ROSInterruptException:
        pass
