#include <ros/ros.h>
#include <ros/console.h>
#include <ros/param.h>
#include <ros/package.h>

#include <boost/bind.hpp>

#include "std_msgs/String.h"
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <image_geometry/stereo_camera_model.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <opencv2/ximgproc.hpp>

#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <dynamic_reconfigure/server.h>
#include <i3dr_stereo_camera/i3DR_DisparityConfig.h>
#include <i3dr_stereo_camera/i3DR_pointCloudConfig.h>
#include <i3dr_stereo_camera/SaveStereo.h>

#include <boost/filesystem.hpp>

#include <stereoMatcher/matcherOpenCVBlock.h>
#include <stereoMatcher/matcherOpenCVSGBM.h>

#ifdef WITH_CUDA
#include <stereoMatcher/matcherOpenCVBlockCuda.h>
#include <stereoMatcher/matcherOpenCVBPCuda.h>
#include <stereoMatcher/matcherOpenCVCSBPCuda.h>
#endif

#ifdef WITH_I3DRSGM
#include <stereoMatcher/matcherI3DRSGM.h>
#endif

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <string>
#include <iostream>
#include <fstream>

using namespace cv;

typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudRGBNormal;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>
    policy_t;

AbstractStereoMatcher *matcher = nullptr;
MatcherOpenCVBlock *block_matcher;
MatcherOpenCVSGBM *sgbm_matcher;
#ifdef WITH_CUDA
MatcherOpenCVBlockCuda *block_cuda_matcher;
MatcherOpenCVBPCuda *bp_cuda_matcher;
MatcherOpenCVCSBPCuda *csbp_cuda_matcher;
#endif
#ifdef WITH_I3DRSGM
MatcherI3DRSGM *i3drsgm_matcher;
#endif

float _depth_max = 10;
float _depth_min = 0;

bool isFirstImagesRecevied = false;

//TODO remove algorithm options if not available
int CV_StereoBM = 0;
int CV_StereoSGBM = 1;
int I3DR_StereoSGM = 2;
int CV_StereoBMCuda = 3;
int CV_StereoBPCuda = 4;
int CV_StereoCSBPCuda = 5;
ros::Publisher _disparity_pub, _rect_l_pub, _rect_r_pub;
std::string _frame_id;
int _stereo_algorithm = CV_StereoBM;

int _min_disparity = 9;
int _disparity_range = 64;
int _correlation_window_size = 15;
int _uniqueness_ratio = 15;
int _texture_threshold = 10;
int _speckle_size = 100;
int _speckle_range = 4;
int _disp12MaxDiff = 0;
float _p1 = 200;
float _p2 = 400;
bool _interp = false;
int _preFilterCap = 31;
int _preFilterSize = 9;

bool _save_points_as_binary = false;

std::string _jr_config_file = "";

cv::Mat _Kl, _Dl, _Rl, _Pl;
cv::Mat _Kr, _Dr, _Rr, _Pr;

cv::Mat _stereo_left, _stereo_right, _stereo_left_rect, _stereo_right_rect, _stereo_disparity;
PointCloudRGB::Ptr _stereo_point_cloud_RGB;
cv::Mat _points_mat;

bool isInitParamConfig = true;
bool isInitParam2Config = true;
int save_index = 0;

ros::Publisher _point_cloud_pub, _point_cloud_normal_pub;

std::string type2str(int type)
{
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch (depth)
  {
  case CV_8U:
    r = "8U";
    break;
  case CV_8S:
    r = "8S";
    break;
  case CV_16U:
    r = "16U";
    break;
  case CV_16S:
    r = "16S";
    break;
  case CV_32S:
    r = "32S";
    break;
  case CV_32F:
    r = "32F";
    break;
  case CV_64F:
    r = "64F";
    break;
  default:
    r = "User";
    break;
  }

  r += "C";
  r += (chans + '0');

  return r;
}

bool save_stereo(i3dr_stereo_camera::SaveStereo::Request &req,
                 i3dr_stereo_camera::SaveStereo::Response &res)
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
    cv::imwrite(req.folderpath + "/" + std::to_string(save_index) + "_l.png", _stereo_left);
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
    cv::imwrite(req.folderpath + "/" + std::to_string(save_index) + "_l_rect.png", _stereo_left_rect);
    cv::imwrite(req.folderpath + "/" + std::to_string(save_index) + "_r_rect.png", _stereo_right_rect);
  }
  if (req.save_disparity)
  {
    if (_stereo_disparity.empty())
    {
      res.res = "Missing disparity image";
      ROS_ERROR("%s", res.res.c_str());
      return false;
    }
    cv::imwrite(req.folderpath + "/" + std::to_string(save_index) + "_disp.png", _stereo_disparity);
  }
  if (req.save_point_cloud)
  {
    if (_stereo_point_cloud_RGB->empty())
    {
      res.res = "Missing point cloud";
      ROS_ERROR("%s", res.res.c_str());
      return false;
    }
    if (_save_points_as_binary)
    {
      pcl::io::savePLYFileBinary(req.folderpath + "/" + std::to_string(save_index) + "_points.ply", *_stereo_point_cloud_RGB);
    }
    else
    {
      pcl::io::savePLYFileASCII(req.folderpath + "/" + std::to_string(save_index) + "_points.ply", *_stereo_point_cloud_RGB);
    }
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

void updateMatcher()
{
  std::cout << "Updating matcher parameters..." << std::endl;

  matcher->setDisparityRange(_disparity_range);
  matcher->setWindowSize(_correlation_window_size);
  matcher->setMinDisparity(_min_disparity);
  matcher->setUniquenessRatio(_uniqueness_ratio);
  matcher->setSpeckleFilterRange(_speckle_range);
  matcher->setSpeckleFilterWindow(_speckle_size);
  matcher->setPreFilterCap(_preFilterCap);
  matcher->setP1(_p1);
  matcher->setP2(_p2);
  matcher->setTextureThreshold(_texture_threshold);
  matcher->setPreFilterSize(_preFilterSize);
  matcher->setInterpolation(_interp);
  //bool occlusion = false;
  //matcher->setOcclusionDetection(_interp);

  std::cout << "Matcher parameters updated." << std::endl;
}

void init_matcher(cv::Size image_size)
{
  std::string empty_str = " ";

  matcher = new MatcherI3DRSGM(_jr_config_file, image_size);
  updateMatcher();
}

//Calculate disparity using left and right images
Mat stereo_match(Mat left_image, Mat right_image)
{
  init_matcher(image_size);
  matcher->setImages(&left_image, &right_image);
  int exitCode = matcher->match();
  if (exitCode == 0)
  {
    matcher->getDisparity(disp);
  }
  else
  {
    ROS_ERROR("Exit code:%d", exitCode);
    ROS_ERROR("Failed to compute stereo match");
    ROS_ERROR("Please check parameters are valid.");
  }

  return disp;
}

void run()
{
  cv::Mat left = cv::zeros(cv::Size(10,10), CV_8UC1);
  cv::Mat right = cv::zeros(cv::Size(10,10), CV_8UC1);
  
  cv::Mat disp;
  cv::Size image_size = cv::Size(left_image.size().width, left_image.size().height);
  cv::Mat(image_size, CV_32F).copyTo(disp);
  
  ROS_INFO("Initalising I3DRSGM...");
  init_matcher(image_size);

  cv::Mat disparity = stereo_match(left, right);
  if (disparity.empty())
  {
    ROS_ERROR("Failed to initalise I3DRSGM disparity");
    return;
  }
  ROS_INFO("I3DRSGM init complete.");
}

int main(int argc, char **argv)
{
    #ifdef WITH_I3DRSGM
        run();
    #else
        std::cerr << "WITH_I3DRSGM build option required."
    #endif
}
