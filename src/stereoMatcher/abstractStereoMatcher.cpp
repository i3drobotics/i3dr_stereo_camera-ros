#include "stereoMatcher/abstractStereoMatcher.h"

AbstractStereoMatcher::AbstractStereoMatcher(std::string &param_file) {
  //this->image_size = image_size;
  //cv::Mat(image_size, CV_32F).copyTo(disparity_buffer);
}

void AbstractStereoMatcher::setImages(cv::Mat *left, cv::Mat *right) {
  this->left = left;
  this->right = right;
}

void AbstractStereoMatcher::getDisparity(cv::Mat &dst) {
  disparity_buffer.copyTo(dst);
  return;
}

void AbstractStereoMatcher::match() {
  forwardMatch();

  disparity_lr.convertTo(disparity_buffer, CV_32F);
}