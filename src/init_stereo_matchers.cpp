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

#include "stereoMatcher/abstractStereoMatcher.h"
#ifdef WITH_I3DRSGM
#include <stereoMatcher/matcherI3DRSGM.h>
#endif

#include <string>
#include <iostream>
#include <fstream>

using namespace cv;

AbstractStereoMatcher *matcher = nullptr;

std::string _jr_config_file = "";

void init_matcher(cv::Size image_size)
{
  #ifdef WITH_I3DRSGM
    matcher = new MatcherI3DRSGM(_jr_config_file, image_size);
  #endif
}

void run()
{
  cv::Mat left = cv::Mat::zeros(cv::Size(10,10), CV_8UC1);
  cv::Mat right = cv::Mat::zeros(cv::Size(10,10), CV_8UC1);
  
  cv::Mat disparity;
  cv::Size image_size = cv::Size(left.size().width, left.size().height);
  cv::Mat(image_size, CV_32F).copyTo(disparity);
  
  ROS_INFO("Initalising matcher...");
  init_matcher(image_size);
  matcher->setImages(&left, &right);
  int exitCode = matcher->match();
  if (exitCode != 0)
  {
    ROS_ERROR("Exit code:%d", exitCode);
    ROS_ERROR("Failed to compute stereo match");
    ROS_ERROR("Please check parameters are valid.");
    return;
  }
  matcher->getDisparity(disparity);
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
