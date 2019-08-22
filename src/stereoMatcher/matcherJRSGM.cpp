#include "stereoMatcher/matcherJRSGM.h"

void MatcherJRSGM::init(void)
{
    std::string param_file = "home/i3dr/Documents/JRIntegration/examples/deimos/JR_configs/match_test.param";
    JR_matcher = new jrsgm(param_file);

    setupDefaultMatcher();

    // Setup for 16-bit disparity
    cv::Mat(image_size, CV_16S).copyTo(disparity_lr);
    cv::Mat(image_size, CV_16S).copyTo(disparity_rl);
}

void MatcherJRSGM::setupDefaultMatcher(void)
{
}

void MatcherJRSGM::forwardMatch()
{
    JR_matcher->compute(*left,*right,disparity_lr);

    //disparity_lr.convertTo(disparity_lr, CV_32F);
}

void MatcherJRSGM::backwardMatch()
{
    //TODO create backward matcher for JRSGM
}