#ifndef MATCHEROPENCVBLOCK_H
#define MATCHEROPENCVBLOCK_H

#include "stereoMatcher/abstractStereoMatcher.h"
#include <opencv2/ximgproc.hpp>

class MatcherOpenCVBlock : public AbstractStereoMatcher
{
public:
    explicit MatcherOpenCVBlock(std::string &param_file, cv::Size _image_size)
        : AbstractStereoMatcher(param_file, _image_size)
    {
        init();
    }

    int forwardMatch(void);
    int backwardMatch(void);

    void setMinDisparity(int min_disparity);
    void setDisparityRange(int disparity_range);
    void setWindowSize(int window_size);
    void setUniquenessRatio(int ratio);
    void setTextureThreshold(int threshold);
    void setSpeckleFilterWindow(int window);
    void setSpeckleFilterRange(int range);
    void setDisp12MaxDiff(int diff);
    void setInterpolation(bool enable);
    void setPreFilterCap(int cap);
    void setPreFilterSize(int size);

    //Not used in OpenCV Block
    void setP1(float p1){};
    void setP2(float p2){};
    void setOcclusionDetection(bool enable){};

private:
    cv::Ptr<cv::StereoBM> matcher;
    void init(void);
    void setupDefaultMatcher(void);
};

#endif // MATCHEROPENCVBLOCK_H
