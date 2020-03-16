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
	stereo_msgs::DisparityImage, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>
	policy_t;

cv::Mat calc_q(cv::Mat m_l, cv::Mat p_r, cv::Mat p_l)
{
	cv::Mat q = cv::Mat::zeros(4, 4, CV_64F);

	double cx = p_l.at<double>(0, 2);
	double cxr = p_r.at<double>(0, 2);
	double cy = p_l.at<double>(1, 2);
	double fx = m_l.at<double>(0, 0);
	double fy = m_l.at<double>(1, 1);

	double p14 = p_r.at<double>(0, 3);
	double T = p14 / fx;
	double q33 = -(cx - cxr) / T;

	q.at<double>(0, 0) = 1.0;
	q.at<double>(0, 3) = -cx;
	q.at<double>(1, 1) = 1.0;
	q.at<double>(1, 3) = -cy;
	q.at<double>(2, 3) = fx;
	q.at<double>(3, 2) = -1.0 / T;
	q.at<double>(3, 3) = q33;

	return q;
}

void cameraInfo_to_KDRP(const sensor_msgs::CameraInfoConstPtr &msg_camera_info, cv::Mat &K, cv::Mat &D, cv::Mat &R, cv::Mat &P)
{
	K = cv::Mat(3, 3, CV_64FC1, (void *)msg_camera_info->K.data());
	D = cv::Mat(1, 5, CV_64FC1, (void *)msg_camera_info->D.data());
	R = cv::Mat(3, 3, CV_64FC1, (void *)msg_camera_info->R.data());
	P = cv::Mat(3, 4, CV_64FC1, (void *)msg_camera_info->P.data());
}

sensor_msgs::Image dispInfoMsg2depthMsg(const stereo_msgs::DisparityImageConstPtr &disparityMsg, const sensor_msgs::CameraInfoConstPtr &camLInfoMsg, const sensor_msgs::CameraInfoConstPtr &camRInfoMsg)
{

	cv::Mat Kl, Dl, Rl, Pl;
	cv::Mat Kr, Dr, Rr, Pr;

	cameraInfo_to_KDRP(camLInfoMsg, Kl, Dl, Rl, Pl);
	cameraInfo_to_KDRP(camRInfoMsg, Kr, Dr, Rr, Pr);

	cv::Mat Q = calc_q(Kl, Pr, Pl);
	cv::Matx44d _Q;
	Q.convertTo(_Q, CV_64F);

	// sensor_msgs::image_encodings::TYPE_32FC1
	cv::Mat disparity(disparityMsg->image.height, disparityMsg->image.width, CV_32FC1, const_cast<uchar *>(disparityMsg->image.data.data()));

	cv::Mat depth32f;
	depth32f = cv::Mat::zeros(disparity.rows, disparity.cols, CV_32F);
	float disp_min_val = disparityMsg->min_disparity;
	float disp_max_val = disparityMsg->max_disparity;

	for (int i = 0; i < disparity.rows; i++)
	{
		for (int j = 0; j < disparity.cols; j++)
		{
			float d = disparity.at<float>(i, j);

			if (d != 10000){

				float x = j - (disparityMsg->image.height/2);
				float y = i - (disparityMsg->image.width/2);

				cv::Vec4d homg_pt = _Q * cv::Vec4d((double)x, (double)y, (double)d, 1.0);

				float depth = (float)homg_pt[2] / (float)homg_pt[3];

				depth32f.at<float>(i, j) = depth;
			}
		}
	}

	// convert to ROS sensor_msg::Image
	cv_bridge::CvImage cvDepth(disparityMsg->header, sensor_msgs::image_encodings::TYPE_32FC1, depth32f);
	sensor_msgs::Image depthMsg;
	cvDepth.toImageMsg(depthMsg);
	return (depthMsg);
}

void callback(const stereo_msgs::DisparityImageConstPtr &disparityMsg, const sensor_msgs::CameraInfoConstPtr &camLInfoMsg, const sensor_msgs::CameraInfoConstPtr &camRInfoMsg)
{
	if (disparityMsg->image.encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) != 0)
	{
		ROS_ERROR("Input type must be disparity=32FC1");
		return;
	}

	bool publish32f = pub32f_.getNumSubscribers();

	if (publish32f)
	{
		sensor_msgs::Image depthMsg = dispInfoMsg2depthMsg(disparityMsg, camLInfoMsg, camRInfoMsg);
		//publish the message
		pub32f_.publish(depthMsg);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "disparity_to_depth");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");

	std::string ns = ros::this_node::getNamespace();

	message_filters::Subscriber<stereo_msgs::DisparityImage> sub_disp(nh, ns + "/disparity", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_l(nh, ns + "/left/camera_info", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_r(nh, ns + "/right/camera_info", 1);

	message_filters::Synchronizer<policy_t> sync(policy_t(10), sub_disp, sub_camera_info_l, sub_camera_info_r);
	sync.registerCallback(boost::bind(&callback, _1, _2, _3));

	image_transport::ImageTransport it(nh);
	pub32f_ = it.advertise("depth", 1);

	ros::spin();
}