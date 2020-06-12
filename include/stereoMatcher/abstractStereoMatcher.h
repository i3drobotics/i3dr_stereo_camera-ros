#ifndef ABSTRACTSTEREOMATCHER_H
#define ABSTRACTSTEREOMATCHER_H

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>

//!  Stereo matcher base class
/*!
  An abstract class to match stereo images. This class should not be used directly,
  instead you should subclass it to add new matchers. The class provides a common interface for image
  matching including threaded execution, forward/back matching and disparity map saving.
*/
class AbstractStereoMatcher
{
public:
  explicit AbstractStereoMatcher(std::string &param_file, cv::Size _image_size);
  ~AbstractStereoMatcher(void){};

  //!  Set images for matching
  /*!
   * @param[in] left Left image
   * @param[in] right Right image
   */
  void setImages(cv::Mat *left, cv::Mat *right);
  virtual void setDownsampleScale(double scale);

  //General paramter functions
  virtual void setDisparityRange(int disparity_range) = 0;
  virtual void setWindowSize(int setWindowSize) = 0;
  virtual void setInterpolation(bool enable) = 0;

  //OpenCV Stereo BM & SGBM only parameter functions
  virtual void setMinDisparity(int min_disparity) = 0;
  virtual void setUniquenessRatio(int ratio) = 0;
  virtual void setSpeckleFilterWindow(int window) = 0;
  virtual void setSpeckleFilterRange(int range) = 0;
  virtual void setDisp12MaxDiff(int diff) = 0;
  virtual void setPreFilterCap(int cap) = 0;
  
  //OpenCV Stereo BM only parameter functions
  virtual void setTextureThreshold(int threshold) = 0;
  virtual void setPreFilterSize(int size) = 0;

  //OpenCV Stereo SGBM only parameter functions
  virtual void setP1(float p1) = 0;
  virtual void setP2(float p2) = 0;

  //JR SGM only parameter functions
  virtual void setOcclusionDetection(bool enable) = 0;

  //! Perform a match with the left image as the reference. This is normally what you want.
  virtual int forwardMatch() = 0;

  //! Perform a match with the right image as the reference.
  virtual int backwardMatch() = 0;

  //! Setup the matcher.
  virtual void init(void) = 0;

  //!  Get the disparity map
  /*!
  * @param[out] dst Output Mat
  */
  void getDisparity(cv::Mat &dst);

  void getBackDisparity(cv::Mat &dist);

  //!  Get a pointer to the left image
  cv::Mat *getLeftImage(void) { return left; }

  //!  Get a pointer to the right image
  cv::Mat *getRighttImage(void) { return right; }

  virtual int match();

protected:
  cv::Mat *left;
  cv::Mat *right;

  cv::Mat disparity_rl;
  cv::Mat disparity_lr;

  cv::Mat disparity_scale;

  cv::Size image_size;

  double downsample_scale = 1;

  int min_disparity = 0;
  int disparity_range = 64;
  int window_size = 9;
  bool interpolate = false;
};
#endif // ABSTRACTSTEREOMATCHER_H