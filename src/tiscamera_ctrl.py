#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty, EmptyResponse, SetBool, SetBoolResponse
from i3dr_stereo_camera.srv import SetInt, SetIntResponse
import tiscamera
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

            self.cam.start_pipeline()

            self.set_property('Trigger Mode', True)

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
                
            #TODO finish adding dynamic config
            srv = Server(tiscamera_settingsConfig, self.dynamic_settings_onChange)

            #create opencv window
            #cv2.namedWindow(str(self.serial),cv2.WINDOW_NORMAL)

        else:
            # required parameters not set
            rospy.logerr("tiscamera_ctrl: Required parameter(s) not set")
            rospy.signal_shutdown(
                "tiscamera_ctrl: Required parameter(s) not set")

    def dynamic_settings_onChange(self, config, level):
        rospy.loginfo("""Reconfigure Request: {Brightness}, {Exposure_Auto}, {Gain_Auto},{Exposure}, {Gain},""".format(**config))

        current_Bright = self.cam.get_property("Brightness")[1]
        current_ExpAuto = self.cam.get_property("Exposure Auto")[1]
        current_GainAuto = self.cam.get_property("Gain Auto")[1]
        current_Exp = self.cam.get_property("Exposure")[1]
        current_Gain = self.cam.get_property("Gain")[1]

        print(self.cam.get_property("Exposure"))
        print(config["Exposure"])

        if current_Bright != config["Brightness"]:
            self.set_property("Brightness",config["Brightness"])

        if current_ExpAuto != config["Exposure_Auto"]:
            self.set_property("Exposure Auto",config["Exposure_Auto"])

        if current_GainAuto != config["Gain_Auto"]:
            self.set_property("Gain Auto",config["Gain_Auto"])

        if current_Exp != config["Exposure"]:
            self.set_property("Exposure",config["Exposure"])

        if current_Gain != config["Gain"]:
            self.set_property("Gain",config["Gain"])

        return config

    def spin(self):
        # start ros node
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            #if CD.newImageReceived is True:
            #    CD.newImageReceived = False
            #    cv2.imshow(str(self.serial), CD.image)
            #else:
            #    print("no image received")
            #cv2.waitKey(1)
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
