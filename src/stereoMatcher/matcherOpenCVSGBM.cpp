#include "stereoMatcher/matcherOpenCVSGBM.h"

void MatcherOpenCVSGBM::init(void)
{

    //TODO convert to non QT
    /*
  QString matcher_parameters = QStandardPaths::AppConfigLocation+"/stereo_bm_params.xml";
  if(QFile(matcher_parameters).exists()){
      try {
        matcher =
            cv::StereoSGBM::load<cv::StereoSGBM>(matcher_parameters.toStdString());
      } catch (cv::Exception& e) {
          qDebug() << "Error loading SGBM matching parameters: " << e.msg.c_str();
          setupDefaultMatcher();
      }
  }else{
      setupDefaultMatcher();
  }
  */
    setupDefaultMatcher();

    // Setup for 16-bit disparity
    cv::Mat(image_size, CV_16S).copyTo(disparity_lr);
    cv::Mat(image_size, CV_16S).copyTo(disparity_rl);
}

void MatcherOpenCVSGBM::setupDefaultMatcher(void)
{
    matcher = cv::StereoSGBM::create(0, 64, 9);
    //setUniquenessRatio(15);
    matcher->setDisp12MaxDiff(-1);
}

void MatcherOpenCVSGBM::forwardMatch()
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
        std::cerr << "Error in SGBM match parameters" << std::endl;
    }
}

void MatcherOpenCVSGBM::backwardMatch()
{
    auto right_matcher = cv::ximgproc::createRightMatcher(matcher);
    right_matcher->compute(*right, *left, disparity_rl);
}

void MatcherOpenCVSGBM::setMinDisparity(int min_disparity) {
    matcher->setMinDisparity(min_disparity);
    this->min_disparity = min_disparity;
}

void MatcherOpenCVSGBM::setDisparityRange(int disparity_range) {
  if ((disparity_range + min_disparity) > image_size.width) return;

  if ((disparity_range > 0) && (disparity_range % 16 == 0)) {
    this->disparity_range = disparity_range;
    matcher->setNumDisparities(disparity_range);
  }
}

void MatcherOpenCVSGBM::setWindowSize(int window_size) {
  this->window_size = window_size;
  matcher->setBlockSize(window_size);
}