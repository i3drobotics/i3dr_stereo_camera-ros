#include "stereoMatcher/matcherOpenCVCSBPCuda.h"

void MatcherOpenCVCSBPCuda::init(void)
{
    setupDefaultMatcher();

    // Setup for 16-bit disparity
    cv::Mat(image_size, CV_16S).copyTo(disparity_lr);
    cv::Mat(image_size, CV_16S).copyTo(disparity_rl);
    cv::cuda::GpuMat(image_size, CV_16S).copyTo(cuda_disp_lr);
    cv::cuda::GpuMat(image_size, CV_16S).copyTo(cuda_disp_rl);
    const int numDevices = cv::cuda::getCudaEnabledDeviceCount();
    std::cerr << "CUDA devices: " << numDevices << std::endl;
    cv::cuda::printCudaDeviceInfo(0);
    cv::cuda::setDevice(0);
}

void MatcherOpenCVCSBPCuda::setupDefaultMatcher(void)
{
    matcher = cv::cuda::createStereoConstantSpaceBP();
}

int MatcherOpenCVCSBPCuda::forwardMatch()
{
    try
    {
        cuda_left.upload(*left);
        cuda_right.upload(*right);
        matcher->compute(cuda_left, cuda_right, cuda_disp_lr);
        cuda_disp_lr.download(disparity_lr);
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
        disparity_lr.convertTo(disparity_lr, CV_32FC1, 16);
        return 0;
    }
    catch (cv::Exception &e)
    {
        const char *err_msg = e.what();
        std::cerr << "Error in OpenCV StereoBMCuda parameters" << std::endl;
        std::cerr << err_msg << std::endl;
        return -1;
    }
}

int MatcherOpenCVCSBPCuda::backwardMatch()
{
    auto right_matcher = cv::ximgproc::createRightMatcher(matcher);
    cuda_left.upload(*left);
    cuda_right.upload(*right);
    right_matcher->compute(cuda_left, cuda_right, cuda_disp_rl);
    cuda_disp_rl.download(disparity_rl);
    return 0;
}

void MatcherOpenCVCSBPCuda::setDisparityRange(int disparity_range)
{
    disparity_range = disparity_range > 0 ? disparity_range : ((image_size.width / 8) + 15) & -16;
    matcher->setNumDisparities(disparity_range);
    this->disparity_range = disparity_range;
}

void MatcherOpenCVCSBPCuda::setMinDisparity(int min_disparity)
{
  matcher->setMinDisparity(min_disparity);
  this->min_disparity = min_disparity;
}

void MatcherOpenCVCSBPCuda::setInterpolation(bool enable)
{
    this->interpolate = enable;
}