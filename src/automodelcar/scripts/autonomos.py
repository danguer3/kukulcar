import rospy
import numpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
estado = 0
lidar = numpy.zeros(5)
def callback(data):
    global estado
    axis_Y = data.axes[7]
    axis_X = data.axes[6]
    if(axis_Y == 1.0):
        estado = 2
    elif(axis_Y == -1.0):
        estado = 1
    elif(axis_X == 1.0):
        estado = 3
    elif(axis_X == -1.0):
        estado = 4
    elif(axis_X == 0.0 and axis_Y == 0.0):
        estado = 0
def callback_scan(data):
    global lidar
    range = data.ranges
    rangos = numpy.asarray(range)
    rangos = numpy.split(rangos,5,axis=0)
    lidar[0] = numpy.mean(rangos[0])
    lidar[1] = numpy.mean(rangos[1])
    lidar[2] = numpy.mean(rangos[2])
    lidar[3] = numpy.mean(rangos[3])
    lidar[4] = numpy.mean(rangos[4])
    #print(rangos_2)

def automodelcar():
    global pub
    global lidar
    global estado
    pub = rospy.Publisher('minirobot', Float32MultiArray, queue_size=1)
    rospy.init_node('automodelcar', anonymous=True)
    rospy.Subscriber("joy", Joy, callback)
    rospy.Subscriber("scan", LaserScan, callback_scan)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        lidar_q = (lidar[0] + lidar[1])/2
        if(lidar_q < 0.5 and estado == 2):
            estado = 0
        pub.publish(Float32MultiArray(data = [estado]))
        print(estado)
        rate.sleep()

if __name__ == '__main__':
    try:
        automodelcar()
    except rospy.ROSInterruptException:
        pass
