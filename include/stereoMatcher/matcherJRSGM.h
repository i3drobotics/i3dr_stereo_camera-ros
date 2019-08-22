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

    void setWindowSize(int window_size);
    void setDisparityRange(int disparity_range);

private:

    void init(void);
    void setupDefaultMatcher(void);

    jrsgm *JR_matcher;
};

#endif // MATCHERJRSGM_H