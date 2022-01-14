#include <ros/ros.h>
#include <ros/console.h>
#include <ros/param.h>
#include <ros/package.h>

#include <boost/bind.hpp>

#include <image_geometry/stereo_camera_model.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <opencv2/ximgproc.hpp>

#include <image_transport/image_transport.h>

#include <boost/filesystem.hpp>

#ifdef WITH_I3DRSGM
#include <stereoMatcher/matcherI3DRSGM.h>
#endif

#include <string>
#include <iostream>
#include <fstream>

using namespace cv;

AbstractStereoMatcher *matcher = nullptr;

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

std::string _jr_config_file = "";

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
  #ifdef WITH_I3DRSGM
    matcher = new MatcherI3DRSGM(_jr_config_file, image_size);
  #endif
  updateMatcher();
}

void run()
{
  cv::Mat left = cv::zeros(cv::Size(10,10), CV_8UC1);
  cv::Mat right = cv::zeros(cv::Size(10,10), CV_8UC1);
  
  cv::Mat disp;
  cv::Size image_size = cv::Size(left_image.size().width, left_image.size().height);
  cv::Mat(image_size, CV_32F).copyTo(disp);
  
  ROS_INFO("Initalising matcher...");
  init_matcher(image_size);
  matcher->setImages(&left_image, &right_image);
  int exitCode = matcher->match();
  if (exitCode != 0)
  {
    ROS_ERROR("Exit code:%d", exitCode);
    ROS_ERROR("Failed to compute stereo match");
    ROS_ERROR("Please check parameters are valid.");
    return
  }
  matcher->getDisparity(disp);
  if (disparity.empty())
  {
    ROS_ERROR("Failed to initalise, empty disparity result.");
    return;
  }
  ROS_INFO("Stereo matchers init complete.");
}

int main(int argc, char **argv)
{
    #ifdef WITH_I3DRSGM
        run();
    #else
        std::cout << "Stereo matchers init only required when built with I3DRSGM support via build option WITH_I3DRSGM." << std::endl;
    #endif
}
