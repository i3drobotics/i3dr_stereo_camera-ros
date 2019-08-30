#include "stereoMatcher/matcherOpenCVBlock.h"

void MatcherOpenCVBlock::init(void)
{
  setupDefaultMatcher();

  // Setup for 16-bit disparity
  cv::Mat(image_size, CV_16S).copyTo(disparity_lr);
  cv::Mat(image_size, CV_16S).copyTo(disparity_rl);
}

void MatcherOpenCVBlock::setupDefaultMatcher(void)
{
  matcher = cv::StereoBM::create(64, 9);
}

int MatcherOpenCVBlock::forwardMatch()
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
    std::cerr << "Error in OpenCV block match parameters" << std::endl;
    return -1;
  }
}

int MatcherOpenCVBlock::backwardMatch()
{
  auto right_matcher = cv::ximgproc::createRightMatcher(matcher);
  right_matcher->compute(*right, *left, disparity_rl);
  return 0;
}

void MatcherOpenCVBlock::setMinDisparity(int min_disparity)
{
  matcher->setMinDisparity(min_disparity);
  this->min_disparity = min_disparity;
}

void MatcherOpenCVBlock::setDisparityRange(int disparity_range)
{
  disparity_range = disparity_range > 0 ? disparity_range : ((image_size.width / 8) + 15) & -16;
  this->disparity_range = disparity_range;
  matcher->setNumDisparities(disparity_range);
}

void MatcherOpenCVBlock::setWindowSize(int window_size)
{
  this->window_size = window_size;
  matcher->setBlockSize(window_size);
}

void MatcherOpenCVBlock::setTextureThreshold(int threshold)
{
  matcher->setTextureThreshold(threshold);
}

void MatcherOpenCVBlock::setUniquenessRatio(int ratio)
{
  matcher->setUniquenessRatio(ratio);
}

void MatcherOpenCVBlock::setSpeckleFilterWindow(int window)
{
  matcher->setSpeckleWindowSize(window);
}

void MatcherOpenCVBlock::setSpeckleFilterRange(int range)
{
  matcher->setSpeckleRange(range);
}

void MatcherOpenCVBlock::setDisp12MaxDiff(int diff)
{
  matcher->setDisp12MaxDiff(diff);
}

void MatcherOpenCVBlock::setInterpolation(bool enable)
{
  this->interpolate = enable;
}

void MatcherOpenCVBlock::setPreFilterCap(int cap){
  matcher->setPreFilterCap(cap);
}

void MatcherOpenCVBlock::setPreFilterSize(int size){
  matcher->setPreFilterSize(size);
}