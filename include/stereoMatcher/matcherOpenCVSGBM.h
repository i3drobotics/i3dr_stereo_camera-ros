#ifndef MATCHEROPENCVSGBM_H
#define MATCHEROPENCVSGBM_H

#include "stereoMatcher/abstractStereoMatcher.h"

class MatcherOpenCVSGBM : public AbstractStereoMatcher
{
public:
  explicit MatcherOpenCVSGBM(std::string &param_file)
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
  cv::Ptr<cv::StereoSGBM> matcher;
  void init(void);
  void setupDefaultMatcher(void);
};

#endif // MATCHEROPENCVSGBM_H