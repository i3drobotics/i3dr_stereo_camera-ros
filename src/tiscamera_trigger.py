#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import Bool
import serial

class serialTrigger:
    def __init__(self,port='/dev/ttyACM0',baudrate=115200,topic='/phobos_nuclear_trigger'):
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(port, baudrate)

        self.pub = rospy.Publisher(topic, Bool)
        rospy.init_node('serialTrigger')

    def spin(self):
        try:
            while not rospy.is_shutdown():
                self.readTrigger()
        except rospy.ROSInterruptException:
            pass

    def readTrigger(self):
        while not rospy.is_shutdown():
            try:
                data = self.ser.readline()
            except serial.SerialException:
                rospy.logerr("Failed to read serial port")
            try:
                split_data = data.rstrip().split(":")
                if (split_data[0] == "Laser"):
                    laser_state = split_data[1]
                    if (laser_state == "ON"):
                        self.pub.publish(True)
                        print("laser:on")
                    elif (laser_state == "OFF"):
                        self.pub.publish(False)
                        print("laser:off")
                    else:
                        rospy.logerr("Invalid serial response for trigger")
            except:
                rospy.logerr("Invalid serial response for trigger")

if __name__ == '__main__':
    port_ = '/dev/ttyACM0'
    baudrate_ = 115200
    topic_ = '/phobos_nuclear_trigger'

    if (rospy.has_param('~Port')):
        port = rospy.get_param('~Port')
        port_ = port
    
    if (rospy.has_param('~Baudrate')):
        baudrate = rospy.get_param('~Baudrate')
        baudrate_ = baudrate

    if (rospy.has_param('~Topic')):
        topic = rospy.get_param('~Topic')
        topic_ = topic

    sTrigger = serialTrigger(port_,baudrate_,topic_)
    sTrigger.spin()
    
