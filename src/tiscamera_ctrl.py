#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty, EmptyResponse, SetBool, SetBoolResponse
from i3dr_stereo_camera.srv import SetInt, SetIntResponse
import tiscamera
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from dynamic_reconfigure.server import Server
from i3dr_stereo_camera.cfg import tiscamera_settingsConfig

class CustomData:
        ''' Example class for user data passed to the on new image callback function
        '''

        def __init__(self, newImageReceived, image):
                self.newImageReceived = newImageReceived
                self.image = image
                self.busy = False

class tiscamera_ctrl(object):
    def __init__(self,CD):
         # initalise ros node
        rospy.init_node('tiscamera_ctrl')

        # check required parameters are set
        if (rospy.has_param('~Serial') and rospy.has_param('~Width') and rospy.has_param('~Height') and rospy.has_param('~FPS')):
            # get width/height topic names from parameters
            self.serial = rospy.get_param('~Serial')
            self.width = rospy.get_param('~Width')
            self.height = rospy.get_param('~Height')
            self.fps = rospy.get_param('~FPS')

            self.camera_info = rospy.get_param('~camera_info',"file:///home/i3dr/.ros/camera_info/cam_info.yaml")
            self.camera_name = rospy.get_param('~camera_name',"cam")
            self.camera_frame = rospy.get_param('~camera_frame',"/cam_optical_frame")

            self.cam = tiscamera.Camera(
                self.serial, self.width, self.height, self.fps, False, False,
                topic_name="tiscamera_topic_%s" % (self.serial), node_name="tiscamera_node_%s" % (self.serial),
                camera_info=self.camera_info,
                camera_name=self.camera_name,
                camera_frame=self.camera_frame)
            self.CD = CD
            self.cam.source

            self.isLaserOn = None

            self.cam.start_pipeline()

            rospy.Service('tiscam_%s_set_brightness' % (self.serial), SetInt, self.set_brightness)
            rospy.Service('tiscam_%s_set_exposure_auto' % (self.serial), SetBool, self.set_exposure_auto)
            rospy.Service('tiscam_%s_set_gain_auto' % (self.serial), SetBool, self.set_gain_auto)
            rospy.Service('tiscam_%s_set_exposure' % (self.serial), SetInt, self.set_exposure)
            rospy.Service('tiscam_%s_set_gain' % (self.serial), SetInt, self.set_gain)

            # set inital camera properties based on parameters
            # initalise rosservice for each camera property given in parameters
            if (rospy.has_param('~Brightness')):
                rospy.loginfo("setting inital brightness")
                brightness = rospy.get_param('~Brightness')
                self.set_property('Brightness', brightness)

            if (rospy.has_param('~Exposure_Auto')):
                rospy.loginfo("setting inital exposure auto")
                exposure_auto = rospy.get_param('~Exposure_Auto')
                self.set_property('Exposure Auto', exposure_auto)
                
            if (rospy.has_param('~Gain_Auto')):
                rospy.loginfo("setting inital gain auto")
                gain_auto = rospy.get_param('~Gain_Auto')
                self.set_property('Gain Auto', gain_auto)
                
            if (rospy.has_param('~Exposure')):
                rospy.loginfo("setting inital exposure")
                exposure = rospy.get_param('~Exposure')
                self.set_property('Exposure', exposure)
                
            if (rospy.has_param('~Gain')):
                rospy.loginfo("setting inital gain")
                gain = rospy.get_param('~Gain')
                self.set_property('Gain', gain)

            if (rospy.has_param('~Trigger')):
                rospy.loginfo("setting inital trigger mode")
                trigger = rospy.get_param('~Trigger')
                self.set_property('Trigger Mode', trigger)
                
            # Start dynamic reconfigure server
            srv = Server(tiscamera_settingsConfig, self.dynamic_settings_onChange)

            self.withLaserWindow = self.camera_name+"_with_laser"
            self.noLaserWindow = self.camera_name+"_no_laser"

            self.cvbridge = CvBridge()
            self.trigger_sub = rospy.Subscriber("/phobos_nuclear_trigger", Bool, self.trigger_callback)
            self.image_sub = rospy.Subscriber("/phobos_nuclear/"+self.camera_name+"/image_raw",Image,self.image_callback)
            self.image_with_laser_pub = rospy.Publisher("/phobos_nuclear/"+self.camera_name+"/image_raw_with_laser", Image,queue_size=10)
            self.image_no_laser_pub = rospy.Publisher("/phobos_nuclear/"+self.camera_name+"/image_raw_no_laser", Image,queue_size=10)

        else:
            # required parameters not set
            rospy.logerr("tiscamera_ctrl: Required parameter(s) not set")
            rospy.signal_shutdown(
                "tiscamera_ctrl: Required parameter(s) not set")

    def start_serial_thread(self):
        pass

    def dynamic_settings_onChange(self, config, level):
        rospy.loginfo("""Reconfigure Request: {Brightness}, {Exposure_Auto}, {Gain_Auto},{Exposure}, {Gain},""".format(**config))

        #TODO add trigger to dynamic reconfigure

        current_Bright = None
        current_Gain = None
        current_Exp = None
        current_ExpAuto = None
        current_GainAuto = None

        try:
            current_Bright = self.cam.get_property("Brightness")[1]
            if current_Bright != config["Brightness"]:
                self.set_property("Brightness",config["Brightness"])
        except:
            print("Failed to set property: Brightness")

        try:
            current_Exp = self.cam.get_property("Exposure")[1]
            if current_Exp != config["Exposure"]:
                self.set_property("Exposure",config["Exposure"])
        except:
            print("Failed to set property: Exposure")

        try:
            current_Gain = self.cam.get_property("Gain")[1]
            if current_Gain != config["Gain"]:
                self.set_property("Gain",config["Gain"])
        except:
            print("Failed to set property: Gain")
        
        try:
            current_ExpAuto = self.cam.get_property("Exposure Auto")[1]
            if current_ExpAuto != config["Exposure_Auto"]:
                self.set_property("Exposure Auto",config["Exposure_Auto"])
        except:
            print("Failed to set property: Exposure Auto")

        try:
            current_GainAuto = self.cam.get_property("Gain Auto")[1]
            if current_GainAuto != config["Gain_Auto"]:
                self.set_property("Gain Auto",config["Gain_Auto"])
        except:
            print("Failed to set property: Gain Auto")

        return config

    def trigger_callback(self, data):
        self.isLaserOn = data.data

    def image_callback(self, data):
        laserOn = self.isLaserOn
        if laserOn:
            print("laser On")
            self.image_with_laser_pub.publish(data)
            try:
                cv_image = self.cvbridge.imgmsg_to_cv2(data, "mono8")
            except CvBridgeError as e:
                print(e)
            resize_img = cv2.resize(cv_image,(640,480))
            cv2.imshow(self.withLaserWindow,resize_img)
            cv2.waitKey(1)
        else:
            print("laser Off")
            self.image_no_laser_pub.publish(data)
            try:
                cv_image = self.cvbridge.imgmsg_to_cv2(data, "mono8")
            except CvBridgeError as e:
                print(e)
            resize_img = cv2.resize(cv_image,(640,480))
            cv2.imshow(self.noLaserWindow,resize_img)
            cv2.waitKey(1)

    def spin(self):
        # start ros node
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            '''
            if CD.newImageReceived is True:
                CD.newImageReceived = False
                print("image recevied")
                #cv2.imshow(str(self.serial), CD.image)
            else:
                print("no image received")
            '''
            rate.sleep()
    
    def stop(self):
        self.cam.stop_pipeline()

    def set_property(self, property_name, data):
        # set camera property using data

        # initalise return values
        success = False
        msg = ""
        try:
            # get current value
            current_value = self.cam.get_property(property_name)
            rospy.loginfo("Current {} value: {}".format(
                property_name, current_value[1]))
            # set property using data value provided
            self.cam.set_property(property_name, data)
            # get value after set to check set was successful
            new_value = self.cam.get_property(property_name)
            msg = "New {} value: {}".format(property_name, new_value[1])
            rospy.loginfo(msg)
        except:
            # failed to set property
            # usually due to missing property in tiscamera
            # check property exists using tcam-ctrl -p <SERIAL>
            success = False
            msg = "[ERROR] Unable to set tiscamera property"
            rospy.logerr(msg)
        return (success, msg)

    def set_brightness(self, req):
        # callback request to set exposure auto
        success, msg = self.set_property("Brightness", req.data)
        return SetIntResponse(success, msg)

    def set_exposure_auto(self, req):
        # callback request to set exposure auto
        success, msg = self.set_property("Exposure Auto", req.data)
        return SetBoolResponse(success, msg)

    def set_exposure(self, req):
        # callback request to set exposure
        success, msg = self.set_property("Exposure", req.data)
        return SetIntResponse(success, msg)

    def set_gain_auto(self, req):
        # callback request to set gain auto
        success, msg = self.set_property("Gain Auto", req.data)
        return SetBoolResponse(success, msg)

    def set_gain(self, req):
        # callback request to set gain
        success, msg = self.set_property("Gain", req.data)
        return SetIntResponse(success, msg)


if __name__ == '__main__':
    # initalise ros node
    CD = CustomData(False, None)
    tiscamera_ctl_node = tiscamera_ctrl(CD)
    try:
        # start ros node
        tiscamera_ctl_node.spin()
    except rospy.ROSInterruptException:
        pass
    tiscamera_ctl_node.stop()
