#include "stereoMatcher/matcherJRSGM.h"

void MatcherJRSGM::init(void)
{
    JR_matcher = new jrsgm(param_file_, image_size_);

    setupDefaultMatcher();
}

void MatcherJRSGM::setupDefaultMatcher(void)
{
    //setDisparityRange(416);
    //setP1(100);
    //setP2(800);
    //setWindowSize(9);
    //setMinDisparity(0);
    //setInterpolation(false);
    //setOcclusionDetection(false);
    //JR_matcher->enableOccInterpol(false);
    //JR_matcher->enableSubpixel(true);
    //JR_matcher->enableTextureDSI(false);
}

int MatcherJRSGM::forwardMatch()
{
    JR_matcher->setImages(*left, *right);
    int exitCode = JR_matcher->compute(disparity_lr);
    if (exitCode == 0)
    {
        disparity_lr.convertTo(disparity_lr, CV_32FC1, -16);
    }
    return exitCode;
}

int MatcherJRSGM::backwardMatch()
{
    int exitCode = JR_matcher->backwardMatch(disparity_rl);
    if (exitCode == 0)
    {
        disparity_rl.convertTo(disparity_rl, CV_32F, -16);
    }
    return exitCode;
}

void MatcherJRSGM::setWindowSize(int window_size)
{
    JR_matcher->setWindowSize(window_size);
}

void MatcherJRSGM::setDisparityRange(int disparity_range)
{
    JR_matcher->setDisparityRange(disparity_range);
}

void MatcherJRSGM::setMinDisparity(int min_disparity)
{
    JR_matcher->setDisparityShift(min_disparity);
}

void MatcherJRSGM::setInterpolation(bool enable)
{
    JR_matcher->enableInterpolation(enable);
}

void MatcherJRSGM::setP1(float P1)
{
    JR_matcher->setP1(P1);
}

void MatcherJRSGM::setP2(float P2)
{
    JR_matcher->setP2(P2);
}

void MatcherJRSGM::setOcclusionDetection(bool enable)
{
    JR_matcher->enableOcclusionDetection(enable);
}

void MatcherJRSGM::setSpeckleFilterWindow(int window){
    JR_matcher->setSpeckleSize(window);
}
void MatcherJRSGM::setSpeckleFilterRange(int range){
    JR_matcher->setSpeckleDifference(range);
}