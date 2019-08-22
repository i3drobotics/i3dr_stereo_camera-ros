#ifndef MATCHEROPENCVBLOCK_H
#define MATCHEROPENCVBLOCK_H

#include "stereoMatcher/abstractStereoMatcher.h"

class MatcherOpenCVBlock : public AbstractStereoMatcher
{
public:
    explicit MatcherOpenCVBlock(std::string &param_file)
        : AbstractStereoMatcher(param_file)
    {
        init();
    }

    void forwardMatch(void);
    void backwardMatch(void);

    void setMinDisparity(int min_disparity);
    void setDisparityRange(int disparity_range);
    void setWindowSize(int window_size);

private:
    cv::Ptr<cv::StereoBM> matcher;
    void init(void);
    void setupDefaultMatcher(void);
};

#endif // MATCHEROPENCVBLOCK_H