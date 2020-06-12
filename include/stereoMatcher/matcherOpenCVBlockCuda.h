#ifndef MATCHEROPENCVBLOCKCUDA_H
#define MATCHEROPENCVBLOCKCUDA_H

#include "stereoMatcher/abstractStereoMatcher.h"
#include "opencv2/cudastereo.hpp"

class MatcherOpenCVBlockCuda : public AbstractStereoMatcher
{
public:
    explicit MatcherOpenCVBlockCuda(std::string &param_file, cv::Size _image_size)
        : AbstractStereoMatcher(param_file, _image_size)
    {
        init();
    }

    int forwardMatch(void);
    int backwardMatch(void);

    void setDisparityRange(int disparity_range);
    void setWindowSize(int window_size);
    void setTextureThreshold(int threshold);
    void setInterpolation(bool enable);
    void setPreFilterCap(int cap);

    //Not used in OpenCV Block CUDA
    void setMinDisparity(int min_disparity){};
    void setUniquenessRatio(int ratio){};
    void setP1(float p1){};
    void setP2(float p2){};
    void setOcclusionDetection(bool enable){};
    void setSpeckleFilterWindow(int window){};
    void setSpeckleFilterRange(int range){};
    void setDisp12MaxDiff(int diff){};
    void setPreFilterSize(int size){};

private:
    cv::Ptr<cv::cuda::StereoBM> matcher;
    cv::cuda::GpuMat cuda_left, cuda_right, cuda_disp_lr, cuda_disp_rl;
    void init(void);
    void setupDefaultMatcher(void);
};

#endif // MATCHEROPENCVBLOCKCUDA_H