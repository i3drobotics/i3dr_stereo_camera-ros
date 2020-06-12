#ifndef MATCHEROPENCVCSBPCUDA_H
#define MATCHEROPENCVCSBPCUDA_H

#include "stereoMatcher/abstractStereoMatcher.h"
#include "opencv2/cudastereo.hpp"

class MatcherOpenCVCSBPCuda : public AbstractStereoMatcher
{
public:
    explicit MatcherOpenCVCSBPCuda(std::string &param_file, cv::Size _image_size)
        : AbstractStereoMatcher(param_file, _image_size)
    {
        init();
    }

    int forwardMatch(void);
    int backwardMatch(void);

    void setMinDisparity(int min_disparity);
    void setDisparityRange(int disparity_range);
    void setInterpolation(bool enable);

    //Not used in OpenCV Block CUDA
    void setWindowSize(int window_size){};
    void setUniquenessRatio(int ratio){};
    void setTextureThreshold(int threshold){};
    void setP1(float p1){};
    void setP2(float p2){};
    void setOcclusionDetection(bool enable){};
    void setSpeckleFilterWindow(int window){};
    void setSpeckleFilterRange(int range){};
    void setDisp12MaxDiff(int diff){};
    void setPreFilterSize(int size){};
    void setPreFilterCap(int cap){};

private:
    cv::Ptr<cv::cuda::StereoConstantSpaceBP> matcher;
    cv::cuda::GpuMat cuda_left, cuda_right, cuda_disp_lr, cuda_disp_rl;
    void init(void);
    void setupDefaultMatcher(void);
};

#endif // MATCHEROPENCVCSBPCUDA_H