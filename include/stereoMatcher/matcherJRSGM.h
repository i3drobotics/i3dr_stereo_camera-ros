#ifndef MATCHERJRSGM_H
#define MATCHERJRSGM_H

#include "stereoMatcher/abstractStereoMatcher.h"
#include "stereoMatcher/jrsgm.h"

class MatcherJRSGM : public AbstractStereoMatcher
{
public:
    explicit MatcherJRSGM(std::string &param_file)
        : AbstractStereoMatcher(param_file)
    {
        init();
    }

    void forwardMatch(void);
    void backwardMatch(void);

    void setWindowSize(int window_size){}; //TODO Impliment functions
    void setDisparityRange(int disparity_range){}; //TODO Impliment functions

    //Not used in JR SGM
    void setMinDisparity(int min_disparity){};
    void setUniquenessRatio(int ratio){};
    void setTextureThreshold(int threshold){};
    void setSpeckleFilterWindow(int window){};
    void setSpeckleFilterRange(int range){};
    void setDisp12MaxDiff(int diff){};
    void setInterpolation(bool enable){};
    void setPreFilterCap(int cap){};
    void setPreFilterSize(int size){};
    void setP1(float p1){};
    void setP2(float p2){};

private:

    void init(void);
    void setupDefaultMatcher(void);

    jrsgm *JR_matcher;
};

#endif // MATCHERJRSGM_H