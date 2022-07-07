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
t_boolean = True
temp_speed = 0.0
def callback_Lidar(data):
    global obst_det,start_time,st_bool,routine
    longitud= len(data.ranges)/4
    lecturas= int(0.4/data.angle_increment) #0.4
    left= longitud-lecturas
    right=longitud+lecturas
    voto = 0
    for i in data.ranges[left:right]: 
        if 0.0 < i < 1.2: # 1.3 
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
def callback_TS(msg):
    global temp_speed
    temp_speed = msg.data
def obstacle_detector():
    global obst_det, start_time, st_bool, routine, angle_temp, t_boolean, temp_speed
    one_time   = 10
    two_time   = 20
    three_time = 30
    four_time  = 16
    five_time  = 13
    time = -1
    print("Initializing node.....")
    rospy.init_node ('obstacle_detector',anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback_Lidar)
    rospy.Subscriber("/temp_data", Float32, callback_Temp)
    rospy.Subscriber("/temp_speed", Float32, callback_TS)
    pub_speed = rospy.Publisher('speed', Float32, queue_size=1)
    pub_steering = rospy.Publisher('steering', Float32, queue_size=1)
    obst_pub = rospy.Publisher("/enable_LT",Bool,queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #print(angle_temp)
        obst_pub.publish(data = routine)
        if routine == True:
            #print("hola")
            #print(obst_det)
            if obst_det == True and time == -1: #Rutina LiDAR
                pub_steering.publish(Float32(data = -30.0))
                pub_speed.publish(Float32(data = 0.33))
                #print("one ", time)
            elif obst_det == False and time == -1: #Escape LiDAR
                #print("two ", time)
                #print("No hay obstaculo")
                #print("Boleano", t_boolean)
                if t_boolean == True:
                    time = 0
                    t_boolean = False
            elif  0 <= time <= one_time: #First time
                #print("three ", time)
                pub_steering.publish(Float32(data = -10.0))
                pub_speed.publish(Float32(data = 0.33))
            elif two_time + one_time >= time > one_time: #second time
                #print("four ", time)
                pub_speed.publish(Float32(data = 0.4))
                pub_steering.publish(Float32(data = 25.0))
            elif three_time + two_time + one_time >= time  > two_time + one_time: # third time
                #print("five ", time)
                pub_steering.publish(Float32(data = angle_temp))
                pub_speed.publish(Float32(data = 0.4)) #antes 0.4 
                print("angulo seguimiento", angle_temp)
            elif four_time+three_time + two_time + one_time >= time  > three_time+two_time + one_time: #Four time
                #print("six ", time)
                pub_speed.publish(Float32(data = 0.35))
                pub_steering.publish(Float32(data = 30.0))
            elif five_time + four_time+three_time + two_time + one_time >= time  > four_time+three_time+two_time + one_time: # five time
                #print("seven ", time)
                pub_steering.publish(Float32(data = -25.0))
                pub_speed.publish(Float32(data = 0.4))
            else:   #termino
                #print("eight ", time)
                pub_steering.publish(Float32(data = 0.0))
                pub_speed.publish(Float32(data = 0.0))
                routine = False
                t_boolean = True
                time = -1
            #print(time)
            if time >= 0:
                time = time+1
        rate.sleep()

if __name__ == '__main__':
    try:
        obstacle_detector()
    except rospy.ROSInterruptException:
        pass
