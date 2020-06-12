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
    std::cout << "Downsample scale: " << downsample_scale << std::endl;
    this->left = new cv::Mat(cv::Size(left->cols*downsample_scale,left->rows*downsample_scale), left->type(), cv::Scalar(0));
    this->right = new cv::Mat(cv::Size(right->cols*downsample_scale,right->rows*downsample_scale), right->type(), cv::Scalar(0));
    std::cout << "New image size: " << this->left->rows <<  this->left->cols << std::endl;
    cv::resize(*left, *this->left, cv::Size(), downsample_scale, downsample_scale, CV_INTER_CUBIC);
    cv::resize(*right, *this->right, cv::Size(), downsample_scale, downsample_scale, CV_INTER_CUBIC);
    this->image_size = this->left->size();
  }
  else
  {
    std::cerr << "Images MUST be the same resolution" << std::endl;
  }
}

void AbstractStereoMatcher::setDownsampleScale(double scale = 1)
{
  downsample_scale = scale;
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
    //cv::resize(disparity_lr, disparity_lr, cv::Size(), 1/downsample_scale, 1/downsample_scale, CV_INTER_CUBIC);
  }
  return exitCode;
}