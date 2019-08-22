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
    //setUniquenessRatio(15);
    matcher->setDisp12MaxDiff(-1);
}

void MatcherOpenCVBlock::forwardMatch()
{
    matcher->setMinDisparity(min_disparity);

    try
    {
        matcher->compute(*left, *right, disparity_lr);
        disparity_lr.convertTo(disparity_lr, CV_32F);
    }
    catch (...)
    {
        std::cerr << "Error in OpenCV block match parameters" << std::endl;
    }
}

void MatcherOpenCVBlock::backwardMatch() {
    auto right_matcher = cv::ximgproc::createRightMatcher(matcher);
    right_matcher->compute(*right, *left, disparity_rl);
}

void MatcherOpenCVBlock::setMinDisparity(int min_disparity) {
  matcher->setMinDisparity(min_disparity);
  this->min_disparity = min_disparity;
}

void MatcherOpenCVBlock::setDisparityRange(int disparity_range) {
  if ((disparity_range + min_disparity) > image_size.width) return;

  if ((disparity_range > 0) && (disparity_range % 16 == 0)) {
    this->disparity_range = disparity_range;
    matcher->setNumDisparities(disparity_range);
  }
}

void MatcherOpenCVBlock::setWindowSize(int window_size) {
  this->window_size = window_size;
  matcher->setBlockSize(window_size);
}