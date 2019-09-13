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

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

image_transport::Publisher pub32f_;
image_transport::Publisher pub16u_;
ros::Subscriber sub_;

float _depth_max = 10;

void callback(const stereo_msgs::DisparityImageConstPtr &disparityMsg)
{
	if (disparityMsg->image.encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) != 0)
	{
		ROS_ERROR("Input type must be disparity=32FC1");
		return;
	}

	bool publish32f = pub32f_.getNumSubscribers();
	bool publish16u = pub16u_.getNumSubscribers();

	if (publish32f || publish16u)
	{
		// sensor_msgs::image_encodings::TYPE_32FC1
		cv::Mat disparity(disparityMsg->image.height, disparityMsg->image.width, CV_32FC1, const_cast<uchar *>(disparityMsg->image.data.data()));

		cv::Mat depth32f;
		cv::Mat depth16u;
		if (publish32f)
		{
			depth32f = cv::Mat::zeros(disparity.rows, disparity.cols, CV_32F);
		}
		if (publish16u)
		{
			depth16u = cv::Mat::zeros(disparity.rows, disparity.cols, CV_16U);
		}
    float disp_min_val = 99999;
    float disp_max_val = 0;
    float depth_max_val = 0;
    float depth_min_val = 99999;
		for (int i = 0; i < disparity.rows; i++)
		{
			for (int j = 0; j < disparity.cols; j++)
			{
				float disparity_value = disparity.at<float>(i, j);
        if (disparity_value < disp_min_val){
          disp_min_val = disparity_value;
        }
        if (disparity_value > disp_max_val){
          disp_max_val = disparity_value;
        }
        //std::cout << "disparity: " << disparity_value << std::endl;
				if (disparity_value > disparityMsg->min_disparity && disparity_value < disparityMsg->max_disparity)
				{
					// baseline * focal / disparity
					float depth = disparityMsg->T * disparityMsg->f / disparity_value;
          
          if (depth < depth_min_val){
            depth_min_val = depth;
          }
          if (depth > depth_max_val){
            depth_max_val = depth;
          }

          //std::cout << "depth:" << depth << std::endl;
					if (depth < depth_max_val && depth > 0) //ignore depth values large than user defined maximum (m)
					{
						if (publish32f)
						{
							depth32f.at<float>(i, j) = depth;
						}
						if (publish16u)
						{
							depth16u.at<unsigned short>(i, j) = (unsigned short)(depth * 1000.0f);
						}
					}
				}
			}
		}

    std::cout << "min disp: " << disp_min_val << std::endl;
    std::cout << "max disp: " << disp_max_val << std::endl;

    std::cout << "min depth: " << depth_min_val << std::endl;
    std::cout << "max depth: " << depth_max_val << std::endl;

		if (publish32f)
		{
			// convert to ROS sensor_msg::Image
			cv_bridge::CvImage cvDepth(disparityMsg->header, sensor_msgs::image_encodings::TYPE_32FC1, depth32f);
			sensor_msgs::Image depthMsg;
			cvDepth.toImageMsg(depthMsg);

			//publish the message
			pub32f_.publish(depthMsg);
		}

		if (publish16u)
		{
			// convert to ROS sensor_msg::Image
			cv_bridge::CvImage cvDepth(disparityMsg->header, sensor_msgs::image_encodings::TYPE_16UC1, depth16u);
			sensor_msgs::Image depthMsg;
			cvDepth.toImageMsg(depthMsg);

			//publish the message
			pub16u_.publish(depthMsg);
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "disparity_to_depth");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");

	float depth_max;

	//[Optional] User defined maximum depth (m)
	if (p_nh.getParam("depth_max", depth_max))
	{
		_depth_max = depth_max;
		ROS_INFO("depth_max: %f", _depth_max);
	}

	image_transport::ImageTransport it(nh);
	pub32f_ = it.advertise("depth", 1);
	pub16u_ = it.advertise("depth_raw", 1);
	sub_ = nh.subscribe("disparity", 1, callback);

	ros::spin();
}