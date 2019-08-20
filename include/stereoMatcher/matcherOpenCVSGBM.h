#ifndef MATCHEROPENCVSGBM_H
#define MATCHEROPENCVSGBM_H

#include "stereoMatcher/abstractStereoMatcher.h"

class MatcherOpenCVSGBM : public AbstractStereoMatcher
{
public:
  explicit MatcherOpenCVSGBM()
      : AbstractStereoMatcher("")
  {
    init();
  }

  void forwardMatch(void);
  void backwardMatch(void);

private:
  cv::Ptr<cv::StereoSGBM> matcher;
  void init(void);
  void setupDefaultMatcher(void);
};

#endif // MATCHEROPENCVSGBM_H