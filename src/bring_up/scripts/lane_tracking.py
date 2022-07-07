#!/usr/bin/env python

""" 
    NODE TO ENABLE CONTROL LAWS (CRUISE SPEED & STEERING ANGLE) FOR LANE TRACKING BEHAVIOR
"""

# LIBRARIES
import rospy
from std_msgs.msg import Float32, Bool, Float64MultiArray
from control_laws import Control
import numpy

# GLOBAL VARIABLES
left_border   = [0.0, 0.0]
right_border  = [0.0, 0.0]
enable_LT     = False
speed         = 0.55 #0.42 #0.55
f_speed       = 0.47 #0.35 #0.47
L_angle       = 22.5 #22 #22.5
#speed = 0.37
#f_speed = 0.3

# LEFT LANE CALLBACK
def callback_left_border(msg):
    global left_border
    left_border = list(msg.data)          # TUPLE TO LIST


# RIGHT LANE CALLBACK
def callback_right_border(msg):
    global right_border
    right_border = list(msg.data)        # TUPLE TO LIST

# ENABLE LANE TRACKIG CALLBACK
def callback_enable_LT(msg):
    global enable_LT
    enable_LT = msg.data


# MAIN FUNCTION
def main():

    global left_border, right_border, enable_LT, speed, L_angle
    left_border = [0.0,0.0]
    right_border = [0.0,0.0]
    # CLASS FOR CONTROL LAWS
    control_LT = Control()

    # INIT NODE
    print('Lane Tracking Node...')
    rospy.init_node('lane_tracking')
    rate = rospy.Rate(10)

    # PARAMS
    if rospy.has_param('/speed'):
        speed = rospy.get_param('/speed')

    # SUBSCRIBERS
    rospy.Subscriber('/left_border', Float64MultiArray, callback_left_border)
    rospy.Subscriber('/right_border', Float64MultiArray, callback_right_border)
    rospy.Subscriber('/enable_LT', Bool, callback_enable_LT)

    # PUBLISHERS
    pub_angle  = rospy.Publisher('/steering', Float32, queue_size=10)
    pub_speed  = rospy.Publisher('/speed', Float32, queue_size=10)
    pub_temp_f = rospy.Publisher('/temp_data', Float32, queue_size=10)
    pub_temp_s = rospy.Publisher('/temp_speed',Float32, queue_size=10)

    while not rospy.is_shutdown():
        control_LT.control_law(left_border, right_border, speed,f_speed)       # COMPUTE CONTROL LAWS
        if control_LT.steering_angle > 0.52:
            angle = L_angle
        elif control_LT.steering_angle < -0.52 :
            angle = -L_angle
        else:
            angle = control_LT.steering_angle*180/numpy.pi
        if enable_LT==False:
            pub_angle.publish(angle)            # PUBLISH STEERING ANGLE
            pub_speed.publish(control_LT.cruise_speed)
        pub_temp_s.publish(control_LT.cruise_speed)
        pub_temp_f.publish(angle)
        rate.sleep()
    counter = 0
    while counter < 10:
        pub_angle.publish(0) 
        pub_speed.publish(0)
        counter += 1    

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


