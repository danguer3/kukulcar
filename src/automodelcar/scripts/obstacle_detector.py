#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_msgs.msg import Float32
obst_det = True
start_time = 0
st_bool = True
routine = False
def callback_Lidar(data):
    global obst_det,time,start_time,st_bool,routine
    longitud= len(data.ranges)/4
    lecturas= int(0.25/data.angle_increment)
    left= longitud-lecturas
    right=longitud+lecturas
    voto = 0
    for i in data.ranges[left:right]: 
        if 0.0 < i < 1.0:
            voto +=1
    if voto > 20:
        #print("Estorba")
        obst_det=True
        routine = True
    else:
        obst_det=False
        #print("Todo libre"))
    
def obstacle_detector():
    global obst_det, time, start_time, st_bool,routine
    one_time   = 1
    two_time   = 3
    three_time = 5
    four_time  = 0
    five_time  = 0
    print("Initializing node.....")
    rospy.init_node ('obstacle_detector',anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback_Lidar)
    pub_speed = rospy.Publisher('speed', Float32, queue_size=1)
    pub_steering = rospy.Publisher('steering', Float32, queue_size=1)
    obst_pub = rospy.Publisher("/enable_LT",Bool,queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #print("Hola")
        obst_pub.publish(data = obst_det)
        time = rospy.get_rostime()
        if routine == True:
            time = rospy.get_rostime()
            #print("hola")
            if st_bool == True:
                start_time = time.secs
                st_bool = False
            print(time.secs - start_time)
            if time.secs - start_time <= one_time: #Fisrs time
                pub_steering.publish(Float32(data = -30.0))
                pub_speed.publish(Float32(data = 0.3))
            elif two_time >= time.secs - start_time > one_time: #second time
                pub_speed.publish(Float32(data = 0.3))
                pub_steering.publish(Float32(data = 0.0))
            elif three_time >= time.secs - start_time > two_time: # third time
                pub_steering.publish(Float32(data = 20.0))
                pub_speed.publish(Float32(data = 0.3))
            elif four_time >= time.secs - start_time > three_time: #Four time
                pub_speed.publish(Float32(data = 0.3))
                pub_steering.publish(Float32(data = 30.0))
            elif five_time >= time.secs - start_time > four_time:
                pub_steering.publish(Float32(data = 30.0))
                pub_speed.publish(Float32(data = 0.3))
            else:
                pub_steering.publish(Float32(data = 0.0))
                pub_speed.publish(Float32(data = 0.0))
                routine = False
                st_bool = True
        rate.sleep()

if __name__ == '__main__':
    try:
        obstacle_detector()
    except rospy.ROSInterruptException:
        pass
