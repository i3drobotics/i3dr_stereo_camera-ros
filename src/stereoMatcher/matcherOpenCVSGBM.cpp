#include "stereoMatcher/matcherOpenCVSGBM.h"

void MatcherOpenCVSGBM::init(void)
{
  setupDefaultMatcher();

  // Setup for 16-bit disparity
  cv::Mat(image_size, CV_16S).copyTo(disparity_lr);
  cv::Mat(image_size, CV_16S).copyTo(disparity_rl);
}

void MatcherOpenCVSGBM::setupDefaultMatcher(void)
{
  matcher = cv::StereoSGBM::create(0, 64, 9);
}

int MatcherOpenCVSGBM::forwardMatch()
{
 try
  {
    matcher->compute(*left, *right, disparity_lr);
    if (interpolate)
    {
      backwardMatch();
      cv::Mat disparity_filter;
      double wls_lambda = 8000;
      double wls_sigma = 1.5;
      auto wls_filter = cv::ximgproc::createDisparityWLSFilter(matcher);
      wls_filter->setLambda(wls_lambda);
      wls_filter->setSigmaColor(wls_sigma);
      wls_filter->filter(disparity_lr, *left, disparity_filter, disparity_rl);
      disparity_rl.copyTo(disparity_lr);
    }
    disparity_lr.convertTo(disparity_lr, CV_32FC1);
    return 0;
  }
  catch (...)
  {
    std::cerr << "Error in SGBM match parameters" << std::endl;
    return -1;
  }
}

int MatcherOpenCVSGBM::backwardMatch()
{
  auto right_matcher = cv::ximgproc::createRightMatcher(matcher);
  right_matcher->compute(*right, *left, disparity_rl);
  return 0;
}

void MatcherOpenCVSGBM::setMinDisparity(int min_disparity)
{
  matcher->setMinDisparity(min_disparity);
  this->min_disparity = min_disparity;
}

void MatcherOpenCVSGBM::setDisparityRange(int disparity_range)
{
  disparity_range = disparity_range > 0 ? disparity_range : ((image_size.width / 8) + 15) & -16;
  this->disparity_range = disparity_range;
  matcher->setNumDisparities(disparity_range);
}

void MatcherOpenCVSGBM::setWindowSize(int window_size)
{
  this->window_size = window_size;
  matcher->setBlockSize(window_size);
}

void MatcherOpenCVSGBM::setUniquenessRatio(int ratio)
{
  matcher->setUniquenessRatio(ratio);
}

void MatcherOpenCVSGBM::setSpeckleFilterWindow(int window)
{
  matcher->setSpeckleWindowSize(window);
}

void MatcherOpenCVSGBM::setSpeckleFilterRange(int range)
{
  matcher->setSpeckleRange(range);
}

void MatcherOpenCVSGBM::setDisp12MaxDiff(int diff)
{
  matcher->setDisp12MaxDiff(diff);
}

void MatcherOpenCVSGBM::setInterpolation(bool enable)
{
  this->interpolate = enable;
}

void MatcherOpenCVSGBM::setP1(float p1)
{
  matcher->setP1(p1);
}

void MatcherOpenCVSGBM::setP2(float p2)
{
  matcher->setP2(p2);
}

void MatcherOpenCVSGBM::setPreFilterCap(int cap){
  matcher->setPreFilterCap(cap);
}