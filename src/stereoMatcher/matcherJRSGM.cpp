#include "stereoMatcher/matcherJRSGM.h"

void MatcherJRSGM::init(void)
{
    JR_matcher = new jrsgm(param_file_);

    setupDefaultMatcher();
}

void MatcherJRSGM::setupDefaultMatcher(void)
{
    setDisparityRange(416);
    setP1(100);
    setP2(800);
    setWindowSize(9);
    setMinDisparity(0);
    setInterpolation(false);
    setOcclusionDetection(false);
    JR_matcher->enableSubpixel(true);
    JR_matcher->enableTextureDSI(true);
}

void MatcherJRSGM::forwardMatch()
{
    JR_matcher->compute(*left,*right,disparity_lr);
    disparity_lr.convertTo(disparity_lr, CV_32F, -16);
}

void MatcherJRSGM::backwardMatch()
{
    JR_matcher->backwardMatch(*left,*right,disparity_rl);
    disparity_rl.convertTo(disparity_rl, CV_32F, -16);
}

void MatcherJRSGM::setWindowSize(int window_size){
    JR_matcher->setWindowSize(window_size);
}

void MatcherJRSGM::setDisparityRange(int disparity_range){
    JR_matcher->setDisparityRange(disparity_range);
}

void MatcherJRSGM::setMinDisparity(int min_disparity){
    JR_matcher->setDisparityShift(min_disparity);
}

void MatcherJRSGM::setInterpolation(bool enable){
    JR_matcher->enableInterpolation(enable);
}

void MatcherJRSGM::setP1(float P1){
    JR_matcher->setP1(P1);
}

void MatcherJRSGM::setP2(float P2){
    JR_matcher->setP2(P2);
}

void MatcherJRSGM::setOcclusionDetection(bool enable){
    JR_matcher->enableOcclusionDetection(enable);
}