#include <ros/ros.h>
#include <ros/console.h>
#include <ros/param.h>

#include <boost/bind.hpp>

#include "std_msgs/String.h"
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <image_geometry/stereo_camera_model.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <i3dr_stereo_camera/SaveRectified.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <string>

using namespace cv;

typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>
    policy_t;

cv::Mat _stereo_left, _stereo_right, _stereo_left_rect, _stereo_right_rect, _stereo_disparity;

ros::Publisher _rect_l_pub, _rect_r_pub;

cv::Mat _Kl, _Dl, _Rl, _Pl;
cv::Mat _Kr, _Dr, _Rr, _Pr;

int save_index = 0;

bool save_stereo(i3dr_stereo_camera::SaveRectified::Request &req,
                 i3dr_stereo_camera::SaveRectified::Response &res)
{
  ROS_INFO("Saving stereo data");
  save_index++;
  if (_stereo_left.empty() || _stereo_right.empty())
  {
    res.res = "Missing stereo images";
    ROS_ERROR("%s", res.res.c_str());
    return false;
  }
  else
  {
    cv::imwrite(req.folderpath + "/" + std::to_string(save_index) +  "_l.png", _stereo_left);
    cv::imwrite(req.folderpath + "/" + std::to_string(save_index) + "_r.png", _stereo_right);
  }

  if (req.save_rectified)
  {
    if (_stereo_left_rect.empty() || _stereo_right_rect.empty())
    {
      res.res = "Missing rectified image";
      ROS_ERROR("%s", res.res.c_str());
      return false;
    }
    cv::imwrite(req.folderpath + "/" + std::to_string(save_index) +  "_l_rect.png", _stereo_left_rect);
    cv::imwrite(req.folderpath + "/" + std::to_string(save_index) +  "_r_rect.png", _stereo_right_rect);
  }

  res.res = "Saved stereo data: " + req.folderpath;
  ROS_INFO("%s", res.res.c_str());
  return true;
}

void cameraInfo_to_KDRP(const sensor_msgs::CameraInfoConstPtr &msg_camera_info, cv::Mat &K, cv::Mat &D, cv::Mat &R, cv::Mat &P)
{
  K = cv::Mat(3, 3, CV_64FC1, (void *)msg_camera_info->K.data());
  D = cv::Mat(1, 5, CV_64FC1, (void *)msg_camera_info->D.data());
  R = cv::Mat(3, 3, CV_64FC1, (void *)msg_camera_info->R.data());
  P = cv::Mat(3, 4, CV_64FC1, (void *)msg_camera_info->P.data());
}

cv::Mat generateQ(double cx, double cy, double cx1, double focal_length, double baseline)
{
  cv::Mat Q(cv::Size(4, 4), CV_64FC1, cv::Scalar(0));
  Q.at<double>(0, 0) = 1.0;
  Q.at<double>(0, 1) = 0.0;
  Q.at<double>(0, 2) = 0.0;
  Q.at<double>(0, 3) = -cx; //cx
  Q.at<double>(1, 0) = 0.0;
  Q.at<double>(1, 1) = 1.0;
  Q.at<double>(1, 2) = 0.0;
  Q.at<double>(1, 3) = -cy; //cy
  Q.at<double>(2, 0) = 0.0;
  Q.at<double>(2, 1) = 0.0;
  Q.at<double>(2, 2) = 0.0;
  Q.at<double>(2, 3) = focal_length; //Focal
  Q.at<double>(3, 0) = 0.0;
  Q.at<double>(3, 1) = 0.0;
  Q.at<double>(3, 2) = 1.0 / baseline;      //-1.0/BaseLine
  Q.at<double>(3, 3) = cx - cx1 / baseline; //cx - cx'/Baseline
  return Q;
}

cv::Mat_<uint8_t> rectify(cv::Mat image, const sensor_msgs::CameraInfoConstPtr &msg_camera_info)
{
  cv::Mat K, D, R, P;
  cameraInfo_to_KDRP(msg_camera_info, K, D, R, P);

  cv::Size resol = cv::Size(image.size().width, image.size().height);

  cv::Mat full_map1, full_map2;

  cv::initUndistortRectifyMap(K, D, R, P, resol,
                              CV_32FC1, full_map1, full_map2);

  cv::Mat_<uint8_t> image_rect;
  cv::remap(image, image_rect, full_map1, full_map2, cv::INTER_CUBIC);

  return (image_rect);
}

void publish_image(ros::Publisher image_pub, const sensor_msgs::ImageConstPtr &msg_image, cv::Mat rect_image)
{
  cv_bridge::CvImage out_msg;
  out_msg.header = msg_image->header;
  out_msg.encoding = sensor_msgs::image_encodings::MONO8;
  out_msg.image = rect_image;

  image_pub.publish(out_msg.toImageMsg());
}

void imageCb(const sensor_msgs::ImageConstPtr &msg_left_image, const sensor_msgs::ImageConstPtr &msg_right_image, const sensor_msgs::CameraInfoConstPtr &msg_left_camera_info, const sensor_msgs::CameraInfoConstPtr &msg_right_camera_info)
{
  image_geometry::StereoCameraModel model_;
  // Update the camera model
  model_.fromCameraInfo(msg_left_camera_info, msg_right_camera_info);

  cv_bridge::CvImagePtr input_image_left, input_image_right;
  input_image_left = cv_bridge::toCvCopy(msg_left_image, "mono8");
  input_image_right = cv_bridge::toCvCopy(msg_right_image, "mono8");

  cv::Mat_<uint8_t> left_rect = rectify(input_image_left->image, msg_left_camera_info);
  cv::Mat_<uint8_t> right_rect = rectify(input_image_right->image, msg_right_camera_info);

  publish_image(_rect_l_pub, msg_left_image, left_rect);
  publish_image(_rect_r_pub, msg_right_image, right_rect);

  _stereo_left = input_image_left->image;
  _stereo_right = input_image_right->image;
  _stereo_left_rect = left_rect;
  _stereo_right_rect = right_rect;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "generate_disparity");
  ros::NodeHandle nh;
  ros::NodeHandle p_nh("~");

  std::string ns = ros::this_node::getNamespace();

  // Publishers creation
  _rect_l_pub = nh.advertise<sensor_msgs::Image>(ns + "/left/image_rect", 1, true);
  _rect_r_pub = nh.advertise<sensor_msgs::Image>(ns + "/right/image_rect", 1, true);

  // Start services
  ros::ServiceServer srv_save_stereo = nh.advertiseService("save_rectified", save_stereo);

  // Subscribers creation.
  message_filters::Subscriber<sensor_msgs::Image> sub_image_l(nh, ns + "/left/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> sub_image_r(nh, ns + "/right/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_l(nh, ns + "/left/camera_info", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_r(nh, ns + "/right/camera_info", 1);

  // Message filter creation.
  message_filters::Synchronizer<policy_t> sync(policy_t(10), sub_image_l, sub_image_r, sub_camera_info_l, sub_camera_info_r);
  sync.registerCallback(boost::bind(&imageCb, _1, _2, _3, _4));

  ros::spin();
}
