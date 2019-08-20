#ifndef MATCHEROPENCVBLOCK_H
#define MATCHEROPENCVBLOCK_H

#include "stereoMatcher/abstractStereoMatcher.h"

class MatcherOpenCVBlock : public AbstractStereoMatcher
{
public:
    explicit MatcherOpenCVBlock()
        : AbstractStereoMatcher("")
    {
        init();
    }

    void forwardMatch(void);
    void backwardMatch(void);

private:
    cv::Ptr<cv::StereoBM> matcher;
    void init(void);
    void setupDefaultMatcher(void);
};

#endif // MATCHEROPENCVBLOCK_H