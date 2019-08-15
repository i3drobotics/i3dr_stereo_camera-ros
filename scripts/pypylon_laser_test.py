## TO USE THIS SCRIPT IN ROS KINETIC
## pypylon is not natively compiled for python 2.7 on x86_64 machines
## Must be compiled from source first (see pypylon readme for details)

from pypylon import pylon
import sys
import os
import cv2
import numpy as np

def main():
    tlFactory = pylon.TlFactory.GetInstance()

    devices = tlFactory.EnumerateDevices()
    if len(devices) == 0:
        raise Exception("No basler cameras found.")

    cameras = pylon.InstantCameraArray(2)

    leftCameraFound = False
    rightCameraFound = False
    camerasFound = False

    for d in devices:
        cam = pylon.InstantCamera(tlFactory.CreateDevice(d))
        dInfo = cam.GetDeviceInfo()
        friendlyName = dInfo.GetFriendlyName()
        manfactureinfo = dInfo.GetModelName()
        if (manfactureinfo == "acA2440-20gm"):
            if (friendlyName == "phobos_nuclear_r (22864912)"):
                leftCameraFound = True
                leftDev = d
                print("left camera found")
                
            elif (friendlyName == "phobos_nuclear_l (22864917)"):
                rightCameraFound = True
                rightDev = d
                print("right camera found")
    if (leftCameraFound and rightCameraFound):
        camerasFound = True
        print("stereo cameras found")

    if (camerasFound):
        for i, cam in enumerate(cameras):
            if (i == 0):
                cam.Attach(tlFactory.CreateDevice(leftDev))
            elif (i == 1):
                cam.Attach(tlFactory.CreateDevice(rightDev))

        cv2.namedWindow('stereoBaslerCameras', cv2.WINDOW_NORMAL)
        cv2.namedWindow('stereoBaslerCameras (With laser)', cv2.WINDOW_NORMAL)
        cv2.namedWindow('stereoBaslerCameras (No laser)', cv2.WINDOW_NORMAL)

        converter = pylon.ImageFormatConverter()
        #converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        converter.OutputPixelFormat = pylon.PixelType_Mono8
        converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
        cameras.StartGrabbing()
        firstImageReceivedR = False
        firstImageReceivedR_laser = False
        firstImageReceivedR_no_laser = False
        firstImageReceivedL = False
        firstImageReceivedL_laser = False
        firstImageReceivedL_no_laser = False

        while cameras.IsGrabbing():
            grabResult = cameras.RetrieveResult(
                5000, pylon.TimeoutHandling_ThrowException)

            if grabResult.GrabSucceeded():
                cameraContextValue = grabResult.GetCameraContext()
                # Access the image data.
                #img = grabResult.Array
                # Convert to opencv image
                image = converter.Convert(grabResult)

                frame = image.GetArray()

                friendlyName = cameras[cameraContextValue].GetDeviceInfo().GetFriendlyName()

                cameras[cameraContextValue].LineSelector.SetValue("Line1")
                line1Status = cameras[cameraContextValue].LineStatus.GetValue()
                print(friendlyName + "Line 1: "+ str(line1Status))
                cameras[cameraContextValue].LineSelector.SetValue("Line3")
                line3Status = cameras[cameraContextValue].LineStatus.GetValue()
                print(friendlyName + "Line 3: "+ str(line3Status))

                if (friendlyName == "phobos_nuclear_r (22864912)"):
                    frame_right = frame
                    firstImageReceivedR = True
                    if (line3Status):
                        frame_right_laser = frame
                        firstImageReceivedR_laser = True
                    else:
                        frame_right_no_laser = frame
                        firstImageReceivedR_no_laser = True
                    
                elif (friendlyName == "phobos_nuclear_l (22864917)"):
                    frame_left = frame
                    firstImageReceivedL = True
                    if (line3Status):
                        frame_left_laser = frame
                        firstImageReceivedL_laser = True
                    else:
                        frame_left_no_laser = frame
                        firstImageReceivedL_no_laser = True

                if (not firstImageReceivedR):
                    frame_right = np.zeros(shape=frame.shape,dtype=np.uint8)

                if (not firstImageReceivedR_laser):
                    frame_right_laser = np.zeros(shape=frame.shape,dtype=np.uint8)

                if (not firstImageReceivedR_no_laser):
                    frame_right_no_laser = np.zeros(shape=frame.shape,dtype=np.uint8)

                if (not firstImageReceivedL):
                    frame_left = np.zeros(shape=frame.shape,dtype=np.uint8)

                if (not firstImageReceivedL_laser):
                    frame_left_laser = np.zeros(shape=frame.shape,dtype=np.uint8)

                if (not firstImageReceivedL_no_laser):
                    frame_left_no_laser = np.zeros(shape=frame.shape,dtype=np.uint8)

            
                frame_joint = np.concatenate((frame_left,frame_right), axis=1)
                frame_joint_with_laser = np.concatenate((frame_left_laser,frame_right_laser), axis=1)
                frame_joint_no_laser = np.concatenate((frame_left_no_laser,frame_right_no_laser), axis=1)

                # Display image
                cv2.imshow('stereoBaslerCameras', frame_joint)
                cv2.imshow('stereoBaslerCameras (With laser)', frame_joint_with_laser)
                cv2.imshow('stereoBaslerCameras (No laser)', frame_joint_no_laser)
                # Close on ESC
                k = cv2.waitKey(1)
                if k == 27:
                    break

        cameras.StopGrabbing()
        #cv2.destroyAllWindows()




























'''
            try:
                leftLine1Status = None
                leftLine3Status = None
                rightLine1Status = None
                rightLine3Status = None

                for cam in cameras:
                    cam.LineSelector.SetValue("Line1")
                    line1Status = cam.LineStatus.GetValue()
                    cam.LineSelector.SetValue("Line3")
                    line3Status = cam.LineStatus.GetValue()
                    if (i == 0):
                        leftLine1Status = line1Status
                        leftLine3Status = line3Status
                    elif (i == 1):
                        rightLine1Status = line1Status
                        rightLine3Status = line3Status

                left_status = "Left: 1-" + str(leftLine1Status) + ", 3-" + str(leftLine3Status)
                right_status = "Right: 1-" + str(rightLine1Status) + ", 3-" + str(rightLine3Status)
                print(left_status)
                print(right_status)

            except KeyboardInterrupt:
                sys.exit()
                '''
   

if __name__=="__main__":
    main()