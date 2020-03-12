#include "stereoMatcher/matcherJRSGM.h"

void MatcherJRSGM::init(void)
{
    JR_matcher = new jrsgm(param_file_, image_size_);

    setupDefaultMatcher();
    cv::Mat(image_size, CV_16S).copyTo(disparity_lr);
    cv::Mat(image_size, CV_16S).copyTo(disparity_rl);
}

void MatcherJRSGM::setupDefaultMatcher(void)
{
    //setOcclusionDetection(false);
    //JR_matcher->enableOccInterpol(false);
    //JR_matcher->enableTextureDSI(false);
    //JR_matcher->enableSubpixel(true);
    //JR_matcher->maxPyramid(4);
}

int MatcherJRSGM::forwardMatch()
{
    JR_matcher->setImages(*left, *right);
    int exitCode = JR_matcher->compute(disparity_lr);
    if (exitCode == 0)
    {
        disparity_lr.convertTo(disparity_lr, CV_32FC1,-16);
    }
    return exitCode;
}

int MatcherJRSGM::backwardMatch()
{
    int exitCode = JR_matcher->backwardMatch(disparity_rl);
    if (exitCode == 0)
    {
        disparity_rl.convertTo(disparity_rl, CV_32FC1, -16);
    }
    return exitCode;
}

void MatcherJRSGM::setWindowSize(int window_size)
{
    this->window_size = window_size;
    JR_matcher->setWindowSize(window_size);
}

void MatcherJRSGM::setDisparityRange(int disparity_range)
{
    this->disparity_range = disparity_range;
    JR_matcher->setDisparityRange(disparity_range);
}

void MatcherJRSGM::setMinDisparity(int min_disparity)
{
    this->min_disparity = min_disparity;
    JR_matcher->setDisparityShift(min_disparity);
}

void MatcherJRSGM::setInterpolation(bool enable)
{
    JR_matcher->enableInterpolation(enable);
    this->interpolate = enable;
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

void MatcherJRSGM::setSpeckleFilterWindow(int window)
{
    JR_matcher->setSpeckleSize(window);
}
void MatcherJRSGM::setSpeckleFilterRange(int range)
{
    JR_matcher->setSpeckleDifference(range);
}
void MatcherJRSGM::setPreFilterCap(int cap){
    JR_matcher->maxPyramid(cap);
}