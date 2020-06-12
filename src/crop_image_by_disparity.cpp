/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
MODIFIED BY: Ben Knight (I3D Robotics)
*/

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <cv_bridge/cv_bridge.h>

image_transport::Publisher pub32f_;

typedef message_filters::sync_policies::ApproximateTime<
	stereo_msgs::DisparityImage, sensor_msgs::Image>
	policy_t;

sensor_msgs::Image maskImageByDisp(const sensor_msgs::ImageConstPtr &camImage, const stereo_msgs::DisparityImageConstPtr &disparityMsg)
{
	// sensor_msgs::image_encodings::TYPE_32FC1
	cv::Mat disparity(disparityMsg->image.height, disparityMsg->image.width, CV_32FC1, const_cast<uchar *>(disparityMsg->image.data.data()));
    cv::Mat image(camImage->height, camImage->width, CV_8UC1, const_cast<uchar *>(camImage->data.data()));
    cv::Mat crop_image = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);

    //TODO check disparity image and camera image are the same size

	for (int i = 0; i < disparity.rows; i++)
	{
		for (int j = 0; j < disparity.cols; j++)
		{
			float d = disparity.at<float>(i, j);

			if (d < 10000){
				crop_image.at<uchar>(i, j) = image.at<uchar>(i,j);
			}
		}
	}

	// convert to ROS sensor_msg::Image
	cv_bridge::CvImage cvImage(camImage->header, sensor_msgs::image_encodings::TYPE_8UC1, crop_image);
	sensor_msgs::Image imageMsg;
	cvImage.toImageMsg(imageMsg);
	return (imageMsg);
}

void callback(const stereo_msgs::DisparityImageConstPtr &disparityMsg, const sensor_msgs::ImageConstPtr &camImage)
{
	if (disparityMsg->image.encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) != 0)
	{
		ROS_ERROR("Input type must be disparity=32FC1");
		return;
	}
    if (camImage->encoding.compare(sensor_msgs::image_encodings::MONO8) != 0)
	{
		ROS_ERROR("Input type must be image=MONO8");
        ROS_ERROR("Input type is: %s",camImage->encoding.c_str());
		return;
	}

	bool publish32f = pub32f_.getNumSubscribers();

	if (publish32f)
	{
		sensor_msgs::Image camCropImage = maskImageByDisp(camImage, disparityMsg);
		//publish the message
		pub32f_.publish(camCropImage);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "crop_image_by_disparity");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");

	std::string ns = ros::this_node::getNamespace();

	message_filters::Subscriber<stereo_msgs::DisparityImage> sub_disp(nh, ns + "/disparity", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_img(nh, ns + "/left/image_rect", 1);

	message_filters::Synchronizer<policy_t> sync(policy_t(10), sub_disp, sub_img);
	sync.registerCallback(boost::bind(&callback, _1, _2));

	image_transport::ImageTransport it(nh);
	pub32f_ = it.advertise(ns + "/left/image_rect_disp_cropped", 1);

	ros::spin();
}