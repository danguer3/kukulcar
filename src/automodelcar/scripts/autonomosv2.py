import rospy
import numpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
estado = [0.0,0.0,0.0]
lidar = numpy.zeros(5)
rango = 0.0
def callback(data):
    global estado
    axisForw = data.axes[2]
    axisBack = data.axes[5]
    axisDir  = data.axes[0]
    estado[0] = axisBack
    estado[1] = axisForw
    estado[2] = axisDir
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
    global estado
    global rango
    pub = rospy.Publisher('minirobot', Float32MultiArray, queue_size=1)
    rospy.init_node('automodelcar', anonymous=True)
    rospy.Subscriber("joy", Joy, callback)
    rospy.Subscriber("scan", LaserScan, callback_scan)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	#print("Dato distancia: ", lidar[3])
        lidar_q = lidar[3]
        print(rango)
	if(0.5< rango < 1.5 and (estado[0] - estado[1]) != 0):
            estado[1] =(-2* (rango - 0.5))+1
        if(0 < rango  < 0.5 and (estado[0] - estado[1]) != 0):
            estado[1] = 1
	if(0.45 < rango  < 0.5 and (estado[0] - estado[1]) != 0):
            estado[0] = -0.3 
        pub.publish(Float32MultiArray(data = estado))
        print("Estado: ", estado)
        rate.sleep()

if __name__ == '__main__':
    try:
        automodelcar()
    except rospy.ROSInterruptException:
        pass
