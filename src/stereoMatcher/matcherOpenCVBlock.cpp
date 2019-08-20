#include "stereoMatcher/matcherOpenCVBlock.h"

void MatcherOpenCVBlock::init(void)
{

    //TODO convert to non QT
    /*
    std::string matcher_parameters = QStandardPaths::AppConfigLocation+"/stereo_bm_params.xml";
    if(QFile(matcher_parameters).exists()){
        try {
            matcher = cv::StereoBM::load<cv::StereoBM>(matcher_parameters.toStdString());
                  } catch (cv::Exception& e) {
            qDebug() << "Error loading block matching parameters" << e.msg.c_str();
            setupDefaultMatcher();
        }
    }else{
        setupDefaultMatcher();
    }
    */

    //TODO setup with ROS params

    setupDefaultMatcher();

    // Setup for 16-bit disparity
    cv::Mat(image_size, CV_16S).copyTo(disparity_lr);
    cv::Mat(image_size, CV_16S).copyTo(disparity_rl);
}

void MatcherOpenCVBlock::setupDefaultMatcher(void)
{
    matcher = cv::StereoBM::create(64, 9);
    //setUniquenessRatio(15);
    matcher->setDisp12MaxDiff(-1);
}

void MatcherOpenCVBlock::forwardMatch()
{
    matcher->setMinDisparity(min_disparity);

    try
    {
        matcher->compute(*left, *right, disparity_lr);

        /*
    if(wls_filter){
        backwardMatch();
        cv::Mat disparity_filter;
        auto wls_filter = cv::ximgproc::createDisparityWLSFilter(matcher);
        wls_filter->setLambda(wls_lambda);
        wls_filter->setSigmaColor(wls_sigma);
        wls_filter->filter(disparity_lr,*left,disparity_filter,disparity_rl);

        disparity_filter.convertTo(disparity_lr, CV_32F);
    }else{
        disparity_lr.convertTo(disparity_lr, CV_32F);
    }
    */
        disparity_lr.convertTo(disparity_lr, CV_32F);
    }
    catch (...)
    {
        std::cerr << "Error in OpenCV block match parameters" << std::endl;
    }
}

void MatcherOpenCVBlock::backwardMatch() {
    auto right_matcher = cv::ximgproc::createRightMatcher(matcher);
    right_matcher->compute(*right, *left, disparity_rl);
}