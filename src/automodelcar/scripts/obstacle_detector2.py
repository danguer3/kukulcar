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
angle_temp = 0.0
def callback_Lidar(data):
    global obst_det,time,start_time,st_bool,routine
    longitud= len(data.ranges)/4
    lecturas= int(0.3/data.angle_increment)
    left= longitud-lecturas
    right=longitud+lecturas
    voto = 0
    for i in data.ranges[left:right]: 
        if 0.0 < i < 1.0:
            voto +=1
    if voto > 3:
        #print("Estorba")
        obst_det=True
        routine = True
    else:
        obst_det=False
        #print("Todo libre"))
def callback_Temp(msg):
    global angle_temp
    angle_temp = msg.data
def obstacle_detector():
    global obst_det, time, start_time, st_bool, routine, angle_temp
    one_time   = 12
    two_time   = 14
    three_time = 30
    four_time  = 14
    five_time  = 15
    print("Initializing node.....")
    rospy.init_node ('obstacle_detector',anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback_Lidar)
    rospy.Subscriber("/temp_data", Float32, callback_Temp)
    pub_speed = rospy.Publisher('speed', Float32, queue_size=1)
    pub_steering = rospy.Publisher('steering', Float32, queue_size=1)
    obst_pub = rospy.Publisher("/enable_LT",Bool,queue_size=10)
    rate = rospy.Rate(10) # 10hz
    time = 0
    while not rospy.is_shutdown():
        print(angle_temp)
        obst_pub.publish(data = routine)
        if routine == True:
            #print("hola")
            if st_bool == True:
                st_bool = False
            elif  time <= one_time: #First time
                pub_steering.publish(Float32(data = -30.0))
                pub_speed.publish(Float32(data = 0.4))
            elif two_time + one_time >= time > one_time: #second time
                pub_speed.publish(Float32(data = 0.4))
                pub_steering.publish(Float32(data = 20.0))
            elif three_time + two_time + one_time >= time  > two_time + one_time: # third time
                pub_steering.publish(Float32(data = angle_temp))
                print(angle_temp)
                pub_speed.publish(Float32(data = 0.4))
            elif four_time+three_time + two_time + one_time >= time  > three_time+two_time + one_time: #Four time
                pub_speed.publish(Float32(data = 0.4))
                pub_steering.publish(Float32(data = 30.0))
            elif five_time + four_time+three_time + two_time + one_time >= time  > four_time+three_time+two_time + one_time:
                pub_steering.publish(Float32(data = -20.0))
                pub_speed.publish(Float32(data = 0.4))
            else:
                pub_steering.publish(Float32(data = 0.0))
                pub_speed.publish(Float32(data = 0.0))
                routine = False
                st_bool = True
                time = 0
            print(time)
            time = time+1
        rate.sleep()

if __name__ == '__main__':
    try:
        obstacle_detector()
    except rospy.ROSInterruptException:
        pass
