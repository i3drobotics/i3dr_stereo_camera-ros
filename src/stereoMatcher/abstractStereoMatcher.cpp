#include "stereoMatcher/abstractStereoMatcher.h"

AbstractStereoMatcher::AbstractStereoMatcher(std::string &param_file) {
  //this->image_size = image_size;
  //cv::Mat(image_size, CV_32F).copyTo(disparity_buffer);
}

void AbstractStereoMatcher::setImages(cv::Mat *left, cv::Mat *right) {
  if (left->size() == right->size()){
    this->left = left;
    this->right = right;
    this->image_size = left->size();
  } else {
    std::cerr << "Images MUST be the same resolution" << std::endl;
  }
}

void AbstractStereoMatcher::getDisparity(cv::Mat &dst) {
  disparity_lr.copyTo(dst);
  return;
}

void AbstractStereoMatcher::getBackDisparity(cv::Mat &dst) {
  disparity_rl.copyTo(dst);
  return;
}

void AbstractStereoMatcher::match() {
  forwardMatch();

  disparity_lr.convertTo(disparity_lr, CV_32F);
}