#include "stereoMatcher/abstractStereoMatcher.h"

AbstractStereoMatcher::AbstractStereoMatcher(std::string &param_file, cv::Size _image_size)
{
  //this->image_size = image_size;
  //cv::Mat(image_size, CV_32F).copyTo(disparity_buffer);
}

void AbstractStereoMatcher::setImages(cv::Mat *left, cv::Mat *right)
{
  if (left->size() == right->size())
  {
    this->left = left;
    this->right = right;
    this->image_size = left->size();
  }
  else
  {
    std::cerr << "Images MUST be the same resolution" << std::endl;
  }
}

void AbstractStereoMatcher::getDisparity(cv::Mat &dst)
{
  disparity_lr.copyTo(dst);
  return;
}

void AbstractStereoMatcher::getBackDisparity(cv::Mat &dst)
{
  disparity_rl.copyTo(dst);
  return;
}

int AbstractStereoMatcher::match()
{
  int exitCode = forwardMatch();
  if (exitCode == 0)
  {
    disparity_lr.convertTo(disparity_lr, CV_32F);
  }
  return exitCode;
}