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

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

#include <cv_bridge/cv_bridge.h>

image_transport::Publisher _depth_pub;
ros::Publisher _point_cloud_pub;
double _depth_max = 100;
double _z_max = 100;
bool _gen_point_cloud = true;

typedef message_filters::sync_policies::ApproximateTime<
	stereo_msgs::DisparityImage, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>
	policy_t;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;


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

void dispInfoMsg2depthMsg(const stereo_msgs::DisparityImageConstPtr &disparityMsg, const sensor_msgs::ImageConstPtr &camImage, const sensor_msgs::CameraInfoConstPtr &camLInfoMsg, const sensor_msgs::CameraInfoConstPtr &camRInfoMsg)
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
	cv::Mat image(camImage->height, camImage->width, CV_8UC1, const_cast<uchar *>(camImage->data.data()));

	PointCloudRGB::Ptr ptCloudTemp(new PointCloudRGB);

	cv::Mat depth32f;
	depth32f = cv::Mat::zeros(disparity.rows, disparity.cols, CV_32F);
	float disp_min_val = disparityMsg->min_disparity;
	float disp_max_val = disparityMsg->max_disparity;

	sensor_msgs::PointCloud2Ptr points_msg = boost::make_shared<sensor_msgs::PointCloud2>();
	points_msg->header = disparityMsg->header;
	points_msg->height = disparityMsg->image.height;
	points_msg->width = disparityMsg->image.width;
	points_msg->is_bigendian = false;
	points_msg->is_dense = false; // there may be invalid points

	sensor_msgs::PointCloud2Modifier pcd_modifier(*points_msg);
  	pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

	sensor_msgs::PointCloud2Iterator<float> iter_x(*points_msg, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(*points_msg, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(*points_msg, "z");
	sensor_msgs::PointCloud2Iterator<uchar> iter_r(*points_msg, "r");
	sensor_msgs::PointCloud2Iterator<uchar> iter_g(*points_msg, "g");
	sensor_msgs::PointCloud2Iterator<uchar> iter_b(*points_msg, "b");

	float max_d = 0;

	for (int i = 0; i < disparity.rows; i++)
	{
		for (int j = 0; j < disparity.cols; j++, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
		{
			float d = disparity.at<float>(i, j);

			if (d < 10000){

				if (d > max_d){
					max_d = d;
				}

				//float x_index = j - (disparityMsg->image.height/2);
				//float y_index = i - (disparityMsg->image.width/2);
				float x_index = j;
				float y_index = i;

				cv::Vec4d homg_pt = _Q * cv::Vec4d((double)x_index, (double)y_index, (double)d, 1.0);
				float z = (float)homg_pt[2] / (float)homg_pt[3];

				if (z <= _depth_max){
					depth32f.at<float>(i, j) = z;
				}
				if (z <= _z_max && _gen_point_cloud){
					float x = (float)homg_pt[0] / (float)homg_pt[3];
					float y = (float)homg_pt[1] / (float)homg_pt[3];
					uchar intensity = image.at<uchar>(i,j);
					*iter_x = x;
					*iter_y = y;
					*iter_z = z;
					*iter_r = intensity;
					*iter_g = intensity;
					*iter_b = intensity;
				}
			}
		}
	}

	std::cerr << "max d: " << max_d << std::endl;

	// convert to ROS sensor_msg::Image
	cv_bridge::CvImage cvDepth(disparityMsg->header, sensor_msgs::image_encodings::TYPE_32FC1, depth32f);
	sensor_msgs::Image depthMsg;
	cvDepth.toImageMsg(depthMsg);

	_depth_pub.publish(depthMsg);
	if (_gen_point_cloud){
		_point_cloud_pub.publish(points_msg);
	}
}

void callback(const stereo_msgs::DisparityImageConstPtr &disparityMsg, const sensor_msgs::ImageConstPtr &camImage, const sensor_msgs::CameraInfoConstPtr &camLInfoMsg, const sensor_msgs::CameraInfoConstPtr &camRInfoMsg)
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
	
	dispInfoMsg2depthMsg(disparityMsg, camImage, camLInfoMsg, camRInfoMsg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "disparity_to_depth");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");

	std::string ns = ros::this_node::getNamespace();

	double depth_max, z_max;
	bool gen_point_cloud;

	if (p_nh.getParam("depth_max", depth_max))
	{
		_depth_max = depth_max;
		ROS_INFO("depth_max: %f", _depth_max);
	}
	if (p_nh.getParam("z_max", z_max))
	{
		_z_max = z_max;
		ROS_INFO("z_max: %f", _z_max);
	}
	if (p_nh.getParam("gen_point_cloud", gen_point_cloud))
	{
		_gen_point_cloud = gen_point_cloud;
		ROS_INFO("gen_point_cloud: %d", _gen_point_cloud);
	}

	message_filters::Subscriber<stereo_msgs::DisparityImage> sub_disp(nh, ns + "/disparity", 1);
	message_filters::Subscriber<sensor_msgs::Image> sub_img(nh, ns + "/left/image_rect", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_l(nh, ns + "/left/camera_info", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_r(nh, ns + "/right/camera_info", 1);

	message_filters::Synchronizer<policy_t> sync(policy_t(100), sub_disp, sub_img, sub_camera_info_l, sub_camera_info_r);
	sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

	image_transport::ImageTransport it(nh);
	_depth_pub = it.advertise(ns + "/depth", 1);
	_point_cloud_pub = nh.advertise<PointCloudRGB>(ns + "/points2", 1);

	ros::spin();
}