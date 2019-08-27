#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import Bool
import serial

class serialTrigger:
    def __init__(self,port='/dev/ttyACM0',baudrate=115200,topic='/phobos_nuclear_trigger'):
        self.port = port
        self.baudrate = baudrate

        self.ser = None

        self.open(port,baudrate)

        self.pub = rospy.Publisher(topic, Bool, queue_size=10)
        
        rospy.init_node('serialTrigger', anonymous=True)

    def spin(self):
        try:
            while not rospy.is_shutdown():
                self.readTrigger()
        except rospy.ROSInterruptException:
            pass

    def open(self,port,baudrate):
        try:
            self.port = port
            self.baudrate = baudrate
            self.ser = serial.Serial(port, baudrate)
        except serial.SerialException:
            self.ser = None
            rospy.logerr("Failed to open serial port")

    def readTrigger(self):
        while not rospy.is_shutdown():
            if (self.ser):
                if self.ser.is_open:
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
                            elif (laser_state == "OFF"):
                                self.pub.publish(False)
                            else:
                                rospy.logerr("Invalid serial response for trigger")
                    except:
                        rospy.logerr("Invalid serial response for trigger")
                else:
                    rospy.logerr("Serial port not open. Trying to reconnect...")
                    self.open(self.port,self.baudrate)
                    rospy.sleep(3.)
            else:
                rospy.logerr("Serial object not created. Tyring to re-create...")
                self.open(self.port,self.baudrate)
                rospy.sleep(3.)

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
    
