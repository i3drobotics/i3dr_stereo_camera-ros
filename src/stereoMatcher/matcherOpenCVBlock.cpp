#include "stereoMatcher/matcherOpenCVBlock.h"

void MatcherOpenCVBlock::init(void)
{
  setupDefaultMatcher();

  // Setup for 16-bit disparity
  cv::Mat(image_size, CV_16S).copyTo(disparity_lr);
  cv::Mat(image_size, CV_16S).copyTo(disparity_rl);
#ifdef WITH_CUDA
  if (enable_gpu)
  {
    cv::cuda::GpuMat(image_size, CV_16S).copyTo(cuda_disp_lr);
    cv::cuda::GpuMat(image_size, CV_16S).copyTo(cuda_disp_rl);
    const int numDevices = cv::cuda::getCudaEnabledDeviceCount();
    std::cerr << "CUDA devices: " << numDevices << std::endl;
    cv::cuda::printCudaDeviceInfo(0);
    cv::cuda::setDevice(0);
  }
#endif
}

void MatcherOpenCVBlock::setupDefaultMatcher(void)
{
#ifdef WITH_CUDA
  if (enable_gpu)
  {
    cuda_matcher = cv::cuda::createStereoBM();
  }
#endif
  matcher = cv::StereoBM::create(64, 9);
}

int MatcherOpenCVBlock::forwardMatch()
{
  try
  {
#ifdef WITH_CUDA
    if (enable_gpu)
    {
      cuda_left.upload(*left);
      cuda_right.upload(*right);
      cuda_matcher->compute(cuda_left, cuda_right, cuda_disp_lr);
      cuda_disp_lr.download(disparity_lr);
      cv::imshow("disparity", (cv::Mat_<uchar>)disparity_lr);
      cv::waitKey(1);
    }
    else
    {
#endif
  matcher->compute(*left, *right, disparity_lr);
#ifdef WITH_CUDA
    }
#endif
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
  catch ( cv::Exception& e )
  {
    const char* err_msg = e.what();
    std::cerr << "Error in OpenCV StereoBM parameters" << std::endl;
    std::cerr << err_msg << std::endl;
    return -1;
  }
}

int MatcherOpenCVBlock::backwardMatch()
{
  auto right_matcher = cv::ximgproc::createRightMatcher(matcher);
#ifdef WITH_CUDA
  if (enable_gpu)
  {
    cuda_left.upload(*left);
    cuda_right.upload(*right);
    right_matcher->compute(cuda_left, cuda_right, cuda_disp_rl);
    cuda_disp_rl.download(disparity_rl);
  }
  else
  {
#endif
  right_matcher->compute(*right, *left, disparity_rl);
#ifdef WITH_CUDA
  }
#endif
  return 0;
}

void MatcherOpenCVBlock::setMinDisparity(int min_disparity)
{
  matcher->setMinDisparity(min_disparity);
  this->min_disparity = min_disparity;
}

void MatcherOpenCVBlock::setDisparityRange(int disparity_range)
{
  #ifdef WITH_CUDA
  if (enable_gpu)
    cuda_matcher->setNumDisparities(disparity_range);
  #endif
  disparity_range = disparity_range > 0 ? disparity_range : ((image_size.width / 8) + 15) & -16;
  matcher->setNumDisparities(disparity_range);
  this->disparity_range = disparity_range;
}

void MatcherOpenCVBlock::setWindowSize(int window_size)
{
  #ifdef WITH_CUDA
  if (enable_gpu)
    cuda_matcher->setBlockSize(window_size);
  #endif
  this->window_size = window_size;
  matcher->setBlockSize(window_size);
}

void MatcherOpenCVBlock::setTextureThreshold(int threshold)
{
  #ifdef WITH_CUDA
  if (enable_gpu)
    cuda_matcher->setTextureThreshold(threshold);
  #endif
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

void MatcherOpenCVBlock::setPreFilterCap(int cap)
{
  #ifdef WITH_CUDA
  if (enable_gpu)
    cuda_matcher->setPreFilterCap(cap);
  #endif
  matcher->setPreFilterCap(cap);
}

void MatcherOpenCVBlock::setPreFilterSize(int size)
{
  #ifdef WITH_CUDA
  if (enable_gpu)
    cuda_matcher->setPreFilterSize(size);
  #endif
  matcher->setPreFilterSize(size);
}