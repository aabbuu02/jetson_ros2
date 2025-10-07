#!/usr/bin/python
import rospy
from std_msgs.msg import Bool
import datetime

file1 = open('/home/nvidia/catkin_WS/src/wms_data/scripts/log.txt', 'a')




def wmsFeedbackCallback(msg):
    global status

    current_time = datetime.datetime.now()

    if(msg.data==1):
        print("data recieved true")
        s="WMS feedback  True\n"
    elif(msg.data==0):
        print("data recieved false")
        s="WMS feedback FALSE\n"
        # logger.error("Exception ddid", exc_info=True)
    file1.write(str(current_time))
    file1.write("      ")
    file1.write(s)



def dataPublisherAndSubscriber():

    rospy.init_node('wms_database_node',anonymous=True)
    rospy.Subscriber("/wms_status", Bool, wmsFeedbackCallback)
    rate=rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
    file1.close()

    


if __name__ == "__main__":
       # while not rospy.is_shutdown():
    try:
        print("wmsDatabase node started")
        dataPublisherAndSubscriber()
    except rospy.ROSInterruptException:
        pass
