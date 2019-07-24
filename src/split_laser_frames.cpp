#include <ros/ros.h>
#include <ros/console.h>
#include <ros/param.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <camera_info_manager/camera_info_manager.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

std::string _non_laser_namespace = "non_laser";
std::string _with_laser_namespace = "with_laser";
ros::Publisher _with_laser_l_img_pub, _with_laser_r_img_pub, _with_laser_l_info_pub, _with_laser_r_info_pub;
ros::Publisher _no_laser_l_img_pub, _no_laser_r_img_pub, _no_laser_l_info_pub, _no_laser_r_info_pub;

int frame_count = 0;
int frame_odd_count = 0;
int frame_even_count = 0;
int frame_dual_count = 0;
float frame_mean_left_odd, frame_mean_left_even, frame_mean_right_odd, frame_mean_right_even = 0;
float frame_sum_left_odd, frame_sum_left_even, frame_sum_right_odd, frame_sum_right_even = 0;
float max_intensity_mean = 0;
float max_intensity_sum = 0;

typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>
    policy_t;

void imageCb(const sensor_msgs::ImageConstPtr &msg_left_image, const sensor_msgs::ImageConstPtr &msg_right_image, const sensor_msgs::CameraInfoConstPtr &msg_left_camera_info, const sensor_msgs::CameraInfoConstPtr &msg_right_camera_info)
{
    //cv_bridge::CvImagePtr input_image_left, input_image_right;
    cv::Mat input_image_left = cv_bridge::toCvCopy(msg_left_image, "mono8")->image;
    cv::Mat input_image_right = cv_bridge::toCvCopy(msg_right_image, "mono8")->image;

    bool isEvenFrame = false;
    int max_frames = 10;

    if (frame_count % 2){
        isEvenFrame = true;
        frame_even_count++;
    } else {
        frame_odd_count++;
    }

    if (frame_dual_count == max_frames){
        if ((frame_mean_left_odd > frame_mean_left_even) && (frame_mean_right_odd > frame_mean_right_even)){
            ROS_INFO("Laser appears in odd images");
        } else if ((frame_mean_left_odd < frame_mean_left_even) && (frame_mean_right_odd < frame_mean_right_even)){
            ROS_INFO("Laser appears in even images");
        } else {
            ROS_ERROR("Unable to distinguish laser in images");
        }       
    }

    if (frame_dual_count < max_frames){
        ROS_INFO("Finding laser in images... %d/%d",frame_dual_count,max_frames);

        cv::Mat thresh_l,thresh_r;

        double minVal_r,minVal_l; 
        double maxVal_r,maxVal_l; 
        cv::Point minLoc; 
        cv::Point maxLoc;

        cv::minMaxLoc( input_image_right, &minVal_r, &maxVal_r, &minLoc, &maxLoc );
        cv::minMaxLoc( input_image_left, &minVal_l, &maxVal_l, &minLoc, &maxLoc );

        max_intensity_sum += maxVal_r + maxVal_l;
        max_intensity_mean = max_intensity_sum/((float)frame_count * 2);

        inRange(input_image_right, 0, max_intensity_mean, thresh_r);
        inRange(input_image_left, 0, max_intensity_mean, thresh_l);

        cv::Mat scaled_image_right, scaled_image_left;

        input_image_right.copyTo(scaled_image_right, thresh_r);
        input_image_left.copyTo(scaled_image_left, thresh_l);

        int left_mean = (cv::mean(scaled_image_left)[0]);
        int right_mean = (cv::mean(scaled_image_right)[0]);

        ROS_INFO("Max intensity: %f",max_intensity_mean);

        if (isEvenFrame){
            frame_sum_left_even = frame_sum_left_even + left_mean;
            frame_sum_right_even = frame_sum_right_even + right_mean;
            frame_mean_left_even = (float)frame_sum_left_even / (float)frame_even_count;
            frame_mean_right_even = (float)frame_sum_right_even / (float)frame_even_count;
        } else {
            frame_sum_left_odd = frame_sum_left_odd + left_mean;
            frame_sum_right_odd = frame_sum_right_odd + right_mean;
            frame_mean_left_odd = (float)frame_sum_left_odd / (float)frame_odd_count;
            frame_mean_right_odd = (float)frame_sum_right_odd / (float)frame_odd_count;
        }

        ROS_INFO("Left odd mean: %f, Left even mean: %f",frame_mean_left_odd,frame_mean_left_even);
        ROS_INFO("Right odd mean: %f, Right even mean: %f",frame_mean_right_odd,frame_mean_right_even);
    } else {
        if ((frame_mean_left_odd > frame_mean_left_even) && (frame_mean_right_odd > frame_mean_right_even)){
            if (isEvenFrame){
                _no_laser_r_info_pub.publish(msg_right_camera_info);
                _no_laser_r_img_pub.publish(msg_right_image);
                _no_laser_l_info_pub.publish(msg_left_camera_info);
                _no_laser_l_img_pub.publish(msg_left_image);
            } else {
                _with_laser_r_info_pub.publish(msg_right_camera_info);
                _with_laser_r_img_pub.publish(msg_right_image);
                _with_laser_l_info_pub.publish(msg_left_camera_info);
                _with_laser_l_img_pub.publish(msg_left_image);
            }
        } else if ((frame_mean_left_odd < frame_mean_left_even) && (frame_mean_right_odd < frame_mean_right_even)){
            if (isEvenFrame){
                _with_laser_r_info_pub.publish(msg_right_camera_info);
                _with_laser_r_img_pub.publish(msg_right_image);
                _with_laser_l_info_pub.publish(msg_left_camera_info);
                _with_laser_l_img_pub.publish(msg_left_image);
            } else {
                _no_laser_r_info_pub.publish(msg_right_camera_info);
                _no_laser_r_img_pub.publish(msg_right_image);
                _no_laser_l_info_pub.publish(msg_left_camera_info);
                _no_laser_l_img_pub.publish(msg_left_image);
            }
        } else {
            return;
        }
    }

    frame_count++;
    if ((frame_odd_count == frame_even_count) && (frame_odd_count != 0)){
        frame_dual_count++;
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "split_laser_frames");
  ros::NodeHandle nh;
  ros::NodeHandle p_nh("~");
  
  std::string ns = ros::this_node::getNamespace();

  std::string non_laser_namespace, with_laser_namespace;

  if (p_nh.getParam("non_laser_namespace", non_laser_namespace))
  {
    _non_laser_namespace = non_laser_namespace;
    ROS_INFO("non_laser_namespace: %s", non_laser_namespace.c_str());
  }

  if (p_nh.getParam("with_laser_namespace", with_laser_namespace))
  {
    _with_laser_namespace = with_laser_namespace;
    ROS_INFO("with_laser_namespace: %s", with_laser_namespace.c_str());
  }
  
  ROS_INFO("%s",ns.c_str());

  _with_laser_l_img_pub = nh.advertise<sensor_msgs::Image>(_with_laser_namespace + "/left/image_raw", 1, true);
  _with_laser_r_img_pub = nh.advertise<sensor_msgs::Image>(_with_laser_namespace + "/right/image_raw", 1, true);
  _with_laser_l_info_pub = nh.advertise<sensor_msgs::CameraInfo>(_with_laser_namespace + "/left/camera_info", 1, true);
  _with_laser_r_info_pub = nh.advertise<sensor_msgs::CameraInfo>(_with_laser_namespace + "/right/camera_info", 1, true);

  _no_laser_l_img_pub = nh.advertise<sensor_msgs::Image>(_non_laser_namespace + "/left/image_raw", 1, true);
  _no_laser_r_img_pub = nh.advertise<sensor_msgs::Image>(_non_laser_namespace + "/right/image_raw", 1, true);
  _no_laser_l_info_pub = nh.advertise<sensor_msgs::CameraInfo>(_non_laser_namespace + "/left/camera_info", 1, true);
  _no_laser_r_info_pub = nh.advertise<sensor_msgs::CameraInfo>(_non_laser_namespace + "/right/camera_info", 1, true);

  message_filters::Subscriber<sensor_msgs::Image> sub_image_l(nh, ns + "/left/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> sub_image_r(nh, ns + "/right/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_l(nh, ns + "/left/camera_info", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_r(nh, ns + "/right/camera_info", 1);

  // Message filter creation.
  message_filters::Synchronizer<policy_t> sync(policy_t(10), sub_image_l, sub_image_r, sub_camera_info_l, sub_camera_info_r);
  sync.registerCallback(boost::bind(&imageCb, _1, _2, _3, _4));

  ros::spin();
}