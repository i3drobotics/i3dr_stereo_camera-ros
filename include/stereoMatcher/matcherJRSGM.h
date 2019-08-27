#ifndef MATCHERJRSGM_H
#define MATCHERJRSGM_H

#include "stereoMatcher/abstractStereoMatcher.h"
#include "stereoMatcher/jrsgm.h"

class MatcherJRSGM : public AbstractStereoMatcher
{
public:
    explicit MatcherJRSGM(std::string &param_file)
        : AbstractStereoMatcher(param_file), param_file_(param_file)
    {
        init();
    }

    void forwardMatch(void);
    void backwardMatch(void);

    void setWindowSize(int window_size);
    void setDisparityRange(int disparity_range);
    void setMinDisparity(int min_disparity);
    void setInterpolation(bool enable);
    void setP1(float p1);
    void setP2(float p2);
    void setOcclusionDetection(bool enable);

    //Not used in JR SGM
    void setUniquenessRatio(int ratio){};
    void setTextureThreshold(int threshold){};
    void setSpeckleFilterWindow(int window){};
    void setSpeckleFilterRange(int range){};
    void setDisp12MaxDiff(int diff){};
    void setPreFilterCap(int cap){};
    void setPreFilterSize(int size){};

private:

    void init(void);
    void setupDefaultMatcher(void);

    std::string param_file_;

    jrsgm *JR_matcher;
};

#endif // MATCHERJRSGM_H