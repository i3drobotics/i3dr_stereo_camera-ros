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
  void setUniquenessRatio(int ratio);
  void setSpeckleFilterWindow(int window);
  void setSpeckleFilterRange(int range);
  void setP1(float p1);
  void setP2(float p2);
  void setDisp12MaxDiff(int diff);
  void setInterpolation(bool enable);
  void setPreFilterCap(int cap);
    
  //Not used in OpenCV SGBM
  void setTextureThreshold(int threshold){};
  void setPreFilterSize(int size){};
  void setOcclusionDetection(bool enable){};

private:
  cv::Ptr<cv::StereoSGBM> matcher;
  void init(void);
  void setupDefaultMatcher(void);
};

#endif // MATCHEROPENCVSGBM_H