#!/usr/bin/env python
import rospy
import numpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

lidar = numpy.zeros(5)
rango = 0.0
speed = 0.0
steering = 0.0
x = 1
def callback(data):
    global axisBack
    global axisForw
    global speed
    global steering
    if data.axes[2] != 0.0 and data.axes[5] != 0.0:
        axisBack = (-data.axes[2]+1)/2
        axisForw = (-data.axes[5]+1)/2
        axisDir  = data.axes[0]
        speed = (axisForw - axisBack)*.3
        steering = -axisDir*30
    
def callback_scan(data):
    global lidar
    global rango
    range = data.ranges
    rangos = numpy.asarray(range)
    rangos = numpy.split(rangos,5,axis=0)
    lidar[0] = numpy.mean(rangos[0])
    lidar[1] = numpy.mean(rangos[1])
    lidar[2] = numpy.mean(rangos[2])
    lidar[3] = numpy.mean(rangos[3])
    lidar[4] = numpy.mean(rangos[4])
    rango = range[329]

def automodelcar():
    global pub
    global lidar
    global speed
    global steering
    global rango
    pub_speed = rospy.Publisher('speed', Float32, queue_size=1)
    pub_steering = rospy.Publisher('steering', Float32, queue_size=1)

    rospy.init_node('automodelcar', anonymous=True)
    rospy.Subscriber("joy", Joy, callback)
    rospy.Subscriber("scan", LaserScan, callback_scan)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #print("Dato distancia: ", lidar[3])
        #lidar_q = lidar[3]
        #print(rango)
        #if(0.5< rango < 1.5 and (estado[0] - estado[1]) != 0):
        #    estado[1] =(-2* (rango - 0.5))+1
        #if(0 < rango  < 0.5 and (estado[0] - estado[1]) != 0):
        #    estado[1] = 1
        #if(0.45 < rango  < 0.5 and (estado[0] - estado[1]) != 0):
        #    estado[0] = -0.3
	
        pub_speed.publish(Float32(data = speed))
        pub_steering.publish(Float32(data = steering))
        #print("Speed", speed)
        #print("Steering: ", steering)
        rate.sleep()

if __name__ == '__main__':
    try:
        automodelcar()
    except rospy.ROSInterruptException:
        pass
