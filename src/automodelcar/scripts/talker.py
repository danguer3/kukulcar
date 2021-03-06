import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
estado = 0
def callback(data):
    axis_Y = data.axes[7]
    axis_X = data.axes[6]
    if(axis_Y == 1.0):
        estado = 1
    elif(axis_Y == -1.0):
        estado = 2
    elif(axis_X == 1.0):
        estado = 3
    elif(axis_X == -1.0):
        estado = 4
    elif(axis_X == 0.0 and axis_Y == 0.0):
        estado = 0
    left_top = Float32MultiArray(data = [estado])
    print(left_top)
    pub.publish(left_top)
    print(estado)

def automodelcar():
    global pub
    pub = rospy.Publisher('minirobot', Float32MultiArray, queue_size=1)
    rospy.init_node('automodelcar', anonymous=True)
    rospy.Subscriber("joy", Joy, callback)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        automodelcar()
    except rospy.ROSInterruptException:
        pass
