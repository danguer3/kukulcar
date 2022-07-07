#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_msgs.msg import Float32
obst_det = False
start_time = 0
st_bool = 0
routine = False
angle_temp = 0.0
def callback_Lidar(data):
    global obst_det,time,start_time,st_bool,routine
    longitud= 0
    lecturas= int(0.3/data.angle_increment)
    if longitud <= 0:
        left = longitud-lecturas+510
    else:
        left = longitud - lecturas
    right=longitud+lecturas
    voto = 0
    for i in data.ranges[left:510]: 
        if 0.0 < i < 0.25:
            voto +=1
    for i in data.ranges[0:right]:
        if 0.0 < i < 0.25:
            voto += 1
    if voto > 3:
        #print("Estorba")
        obst_det=True
        routine = True
    else:
        obst_det=False
        #print("Todo libre")
def callback_Temp(msg):
    global angle_temp
    angle_temp = msg.data
def obstacle_detector():
    global obst_det, time, start_time, st_bool, routine, angle_temp
    one_time   = 0
    two_time   = 10
    three_time = 20
    four_time  = 25
    five_time  = 5
    six_time   = 15
    seven_time = 7
    print("Initializing node.....")
    rospy.init_node ('obstacle_detector',anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback_Lidar)
    rospy.Subscriber("/temp_data", Float32, callback_Temp)
    pub_speed = rospy.Publisher('speed', Float32, queue_size=1)
    pub_steering = rospy.Publisher('steering', Float32, queue_size=1)
    obst_pub = rospy.Publisher("/enable_LT",Bool,queue_size=10)
    rate = rospy.Rate(10) # 10hz
    time = 0
    parking = 0
    change = True
    activator_rout = False
    while not rospy.is_shutdown():
        obst_pub.publish(data = routine)
        if routine == True and activator_rout == False:
            pub_speed.publish(Float32(data = 0.32))
            pub_steering.publish(Float32(data = angle_temp))
        if obst_det == True and activator_rout == False:
            if change == True:
                parking += 1
                change = False
        else:
            if change == False:
                parking += 1
            change = True
        print("Parking",parking,"Change",change,"obstaculo",obst_det)
        if parking == 4:
            activator_rout = True
            if  time <= one_time: #First time
                pub_steering.publish(Float32(data = 0.0))
                pub_speed.publish(Float32(data = 0.0))
            elif two_time + one_time >= time > one_time: #second time
                pub_speed.publish(Float32(data = -0.35))
                pub_steering.publish(Float32(data = 0.0))
            elif three_time + two_time + one_time >= time  > two_time + one_time: # third time
                pub_steering.publish(Float32(data = 30))
                pub_speed.publish(Float32(data = -0.35))
            elif four_time+three_time + two_time + one_time >= time  > three_time+two_time + one_time: #Four time
                pub_speed.publish(Float32(data = -0.35))
                pub_steering.publish(Float32(data = -25.0))
            elif five_time + four_time+three_time + two_time + one_time >= time  > four_time+three_time+two_time + one_time:
                pub_steering.publish(Float32(data = 0.0))
                pub_speed.publish(Float32(data = -0.35))
            elif six_time + five_time + four_time+three_time + two_time + one_time >= time  > five_time + four_time+three_time+two_time + one_time:
                pub_steering.publish(Float32(data = 15.0))
                pub_speed.publish(Float32(data = 0.40))
            elif seven_time + six_time + five_time + four_time+three_time + two_time + one_time >= time  > six_time +five_time + four_time+three_time+two_time + one_time:
                pub_steering.publish(Float32(data = 0.0))
                pub_speed.publish(Float32(data = -0.35))
            else:
                pub_steering.publish(Float32(data = 0.0))
                pub_speed.publish(Float32(data = 0.0))
                #routine = False
                parking = 0
                time = 0
                #activator_rout = False
            print(time)
            time = time+1
        rate.sleep()

if __name__ == '__main__':
    try:
        obstacle_detector()
    except rospy.ROSInterruptException:
        pass
