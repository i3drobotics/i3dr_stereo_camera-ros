#include <ros/ros.h>
#include <ros/console.h>
#include <ros/param.h>
#include <ros/package.h>

#include <boost/bind.hpp>

#include "std_msgs/String.h"
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <image_geometry/stereo_camera_model.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

#include <opencv2/ximgproc.hpp>

#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <dynamic_reconfigure/server.h>
#include <i3dr_stereo_camera/i3DR_DisparityConfig.h>
#include <i3dr_stereo_camera/SaveStereo.h>

#include <boost/filesystem.hpp>

#include <stereoMatcher/matcherOpenCVBlock.h>
#include <stereoMatcher/matcherOpenCVSGBM.h>

//#define ENABLE_I3DR_ALG ON //TODO: REMOVE THIS

#ifdef ENABLE_I3DR_ALG
#include <stereoMatcher/matcherJRSGM.h>
#endif

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

#include <string>

using namespace cv;

typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudRGBNormal;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>
    policy_t;

AbstractStereoMatcher *matcher = nullptr;
MatcherOpenCVBlock *block_matcher;
MatcherOpenCVSGBM *sgbm_matcher;
MatcherJRSGM *jrsgm_matcher;

int CV_StereoBM = 0;
int CV_StereoSGBM = 1;
int JR_StereoSGM = 2;
ros::Publisher _disparity_pub, _rect_l_pub, _rect_r_pub;
std::string _frame_id;
int _stereo_algorithm = CV_StereoBM;

int _min_disparity = 9;
int _disparity_range = 64;
int _correlation_window_size = 15;
int _uniqueness_ratio = 15;
int _texture_threshold = 10;
int _speckle_size = 100;
int _speckle_range = 4;
int _disp12MaxDiff = 0;
float _p1 = 200;
float _p2 = 400;
bool _interp = false;
int _preFilterCap = 31;
int _preFilterSize = 9;

std::string _jr_config_file = "/home/i3dr/i3dr_tools_ros/i3dr_tools_ros_WS/src/i3dr_cameras/i3dr_stereo_camera/ini/JR_matchingparam_without_interpolation.cfg";

cv::Mat _Kl, _Dl, _Rl, _Pl;
cv::Mat _Kr, _Dr, _Rr, _Pr;

cv::Mat _stereo_left, _stereo_right, _stereo_left_rect, _stereo_right_rect, _stereo_disparity;
PointCloudRGB::Ptr _stereo_point_cloud_RGB;

bool isInitParamConfig = true;
int save_index = 0;

ros::Publisher _point_cloud_pub, _point_cloud_normal_pub;

bool save_stereo(i3dr_stereo_camera::SaveStereo::Request &req,
                 i3dr_stereo_camera::SaveStereo::Response &res)
{
  ROS_INFO("Saving stereo data");
  save_index++;
  if (_stereo_left.empty() || _stereo_right.empty())
  {
    res.res = "Missing stereo images";
    ROS_ERROR("%s", res.res.c_str());
    return false;
  }
  else
  {
    cv::imwrite(req.folderpath + "/" + std::to_string(save_index) + "_l.png", _stereo_left);
    cv::imwrite(req.folderpath + "/" + std::to_string(save_index) + "_r.png", _stereo_right);
  }

  if (req.save_rectified)
  {
    if (_stereo_left_rect.empty() || _stereo_right_rect.empty())
    {
      res.res = "Missing rectified image";
      ROS_ERROR("%s", res.res.c_str());
      return false;
    }
    cv::imwrite(req.folderpath + "/" + std::to_string(save_index) + "_l_rect.png", _stereo_left_rect);
    cv::imwrite(req.folderpath + "/" + std::to_string(save_index) + "_r_rect.png", _stereo_right_rect);
  }
  if (req.save_disparity)
  {
    if (_stereo_disparity.empty())
    {
      res.res = "Missing disparity image";
      ROS_ERROR("%s", res.res.c_str());
      return false;
    }
    cv::imwrite(req.folderpath + "/" + std::to_string(save_index) + "_disp.png", _stereo_disparity);
  }
  if (req.save_point_cloud)
  {
    if (_stereo_point_cloud_RGB->empty())
    {
      res.res = "Missing point cloud";
      ROS_ERROR("%s", res.res.c_str());
      return false;
    }
    pcl::io::savePLYFileBinary(req.folderpath + "/" + std::to_string(save_index) + "_points.ply", *_stereo_point_cloud_RGB);
  }

  res.res = "Saved stereo data: " + req.folderpath;
  ROS_INFO("%s", res.res.c_str());
  return true;
}

void cameraInfo_to_KDRP(const sensor_msgs::CameraInfoConstPtr &msg_camera_info, cv::Mat &K, cv::Mat &D, cv::Mat &R, cv::Mat &P)
{
  K = cv::Mat(3, 3, CV_64FC1, (void *)msg_camera_info->K.data());
  D = cv::Mat(1, 5, CV_64FC1, (void *)msg_camera_info->D.data());
  R = cv::Mat(3, 3, CV_64FC1, (void *)msg_camera_info->R.data());
  P = cv::Mat(3, 4, CV_64FC1, (void *)msg_camera_info->P.data());
}

//Convert disparity image from opencv Mat to PCL Point Cloud XYZRGB
PointCloudRGB::Ptr Mat2PCL(cv::Mat image, cv::Mat coords, PointCloudRGBNormal::Ptr normals)
{

  PointCloudRGB::Ptr ptCloudTemp(new PointCloudRGB);
  PointCloudRGBNormal::Ptr ptCloudNormals(new PointCloudRGBNormal);

  pcl::PointXYZRGB point;
  pcl::PointXYZRGBNormal pNormal;
  uint32_t rgb = 0;
  uchar col = 0;

  point.x = 0;
  point.y = 0;
  point.z = 0;

  pNormal.x = 0;
  pNormal.y = 0;
  pNormal.z = 0;

  pNormal.normal_x = 0;
  pNormal.normal_y = 0;
  pNormal.normal_z = 0;

  rgb = ((int)255) << 16 | ((int)255) << 8 | ((int)255);
  point.rgb = *reinterpret_cast<float *>(&rgb);

  pNormal.rgb = point.rgb;

  ptCloudTemp->points.push_back(point);
  ptCloudNormals->points.push_back(pNormal);

  for (int i = 1; i < coords.rows - 1; i++)
  {
    float *reconst_ptr = coords.ptr<float>(i);
    uchar *rgb_ptr = image.ptr<uchar>(i);

    if (!rgb_ptr || !reconst_ptr)
      return (ptCloudTemp);

    for (int j = 1; j < coords.cols - 1; j++)
    {
      if (rgb_ptr[j] == 0)
        continue;

      point.x = reconst_ptr[3 * j];
      point.y = reconst_ptr[3 * j + 1];
      point.z = reconst_ptr[3 * j + 2];

      if (abs(point.x) > 10)
        continue;
      if (abs(point.y) > 10)
        continue;
      if (abs(point.z) > 10)
        continue;
      col = rgb_ptr[j];

      rgb = ((int)col) << 16 | ((int)col) << 8 | ((int)col);
      point.rgb = *reinterpret_cast<float *>(&rgb);

      //normals
      float dzdx = (coords.at<float>(i + 1, j) - coords.at<float>(i - 1, j)) / 2.0;
      float dzdy = (coords.at<float>(i, j + 1) - coords.at<float>(i, j - 1)) / 2.0;

      cv::Vec3f d(-dzdx, -dzdy, 1.0f);

      cv::Vec3f n = normalize(d);

      pNormal.x = point.x;
      pNormal.y = point.y;
      pNormal.z = point.z;

      pNormal.rgb = point.rgb;

      pNormal.normal_x = n[0];
      pNormal.normal_y = n[1];
      pNormal.normal_z = n[2];

      ptCloudNormals->points.push_back(pNormal);

      ptCloudTemp->points.push_back(point);
    }
  }
  pcl::copyPointCloud(*ptCloudNormals, *normals);
  return (ptCloudTemp);
}

//Calculate disparity using left and right images
Mat stereo_match(Mat left_image, Mat right_image)
{
  cv::Mat disp;
  cv::Size image_size = cv::Size(left_image.size().width, left_image.size().height);

  // Setup for 16-bit disparity
  cv::Mat(image_size, CV_16S).copyTo(disp);

  matcher->setImages(&left_image, &right_image);
  matcher->setDisparityRange(_disparity_range);
  matcher->setWindowSize(_correlation_window_size);
  matcher->setInterpolation(_interp);
  if (_stereo_algorithm == CV_StereoBM || _stereo_algorithm == CV_StereoSGBM)
  {
    //Functions unique to OpenCV Stereo BM & SGBM
    matcher->setMinDisparity(_min_disparity);
    matcher->setUniquenessRatio(_uniqueness_ratio);
    matcher->setSpeckleFilterRange(_speckle_range);
    matcher->setSpeckleFilterWindow(_speckle_size);
    matcher->setPreFilterCap(_preFilterCap);

    if (_stereo_algorithm == CV_StereoBM)
    {
      //Functions unique to OpenCV Stereo BM
      matcher->setTextureThreshold(_texture_threshold);
      matcher->setPreFilterSize(_preFilterSize);
    }
    else
    {
      //Functions unique to OpenCV Stereo SGBM
      matcher->setP1(_p1);
      matcher->setP2(_p2);
    }
  }
  else if (_stereo_algorithm == JR_StereoSGM)
  {
    //Functions unique to JR Stereo SGM
  }

  matcher->match();
  matcher->getDisparity(disp);

  return disp;
}

void publish_disparity(cv::Mat disparity, int min_disparity, int disparity_range, const sensor_msgs::CameraInfoConstPtr &msg_left_camera_info, const sensor_msgs::CameraInfoConstPtr &msg_right_camera_info)
{
  stereo_msgs::DisparityImage disp_msg;
  disp_msg.min_disparity = min_disparity;
  disp_msg.max_disparity = min_disparity + disparity_range - 1;

  cv::Mat Kl, Dl, Rl, Pl;
  cv::Mat Kr, Dr, Rr, Pr;

  cameraInfo_to_KDRP(msg_left_camera_info, Kl, Dl, Rl, Pl);
  cameraInfo_to_KDRP(msg_right_camera_info, Kr, Dr, Rr, Pr);

  // should be safe
  disp_msg.valid_window.x_offset = 0;
  disp_msg.valid_window.y_offset = 0;
  disp_msg.valid_window.width = 0;
  disp_msg.valid_window.height = 0;
  disp_msg.T = 0;
  disp_msg.f = 0;
  disp_msg.delta_d = 0;
  disp_msg.header.stamp = ros::Time::now();
  disp_msg.header.frame_id = _frame_id;

  sensor_msgs::Image &dimage = disp_msg.image;
  dimage.width = disparity.size().width;
  dimage.height = disparity.size().height;
  dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  dimage.step = dimage.width * sizeof(float);
  dimage.data.resize(dimage.step * dimage.height);
  cv::Mat_<float> dmat(dimage.height, dimage.width, (float *)&dimage.data[0], dimage.step);

  static const int DPP = 16; // disparities per pixel
  static const double inv_dpp = 1.0 / DPP;

  double cxR = Kr.at<double>(0, 2);
  double cxL = Kl.at<double>(0, 2);
  double f = Kl.at<double>(0, 0);
  double T = Pr.at<double>(0, 3) / Kr.at<double>(0, 0); //baseline = P14 / f

  disparity.convertTo(dmat, dmat.type(), inv_dpp, -(cxL - cxR));
  ROS_ASSERT(dmat.data == &dimage.data[0]);

  disp_msg.delta_d = inv_dpp;

  disp_msg.f = f;
  disp_msg.T = T;

  double cx_l = msg_left_camera_info->K[2];
  double cx_r = msg_right_camera_info->K[2];
  if (cx_l != cx_r)
  {
    cv::Mat_<float> disp_image(disp_msg.image.height, disp_msg.image.width,
                               reinterpret_cast<float *>(&disp_msg.image.data[0]),
                               disp_msg.image.step);
    cv::subtract(disp_image, cv::Scalar(cx_l - cx_r), disp_image);
  }

  _disparity_pub.publish(disp_msg);
}

cv::Mat_<uint8_t> rectify(cv::Mat image, const sensor_msgs::CameraInfoConstPtr &msg_camera_info)
{
  cv::Mat K, D, R, P;
  cameraInfo_to_KDRP(msg_camera_info, K, D, R, P);

  cv::Size resol = cv::Size(image.size().width, image.size().height);

  cv::Mat full_map1, full_map2;

  cv::initUndistortRectifyMap(K, D, R, P, resol,
                              CV_32FC1, full_map1, full_map2);

  cv::Mat_<uint8_t> image_rect;
  cv::remap(image, image_rect, full_map1, full_map2, cv::INTER_CUBIC);

  return (image_rect);
}

void publish_image(ros::Publisher image_pub, const sensor_msgs::ImageConstPtr &msg_image, cv::Mat rect_image)
{
  cv_bridge::CvImage out_msg;
  out_msg.header = msg_image->header;
  out_msg.encoding = sensor_msgs::image_encodings::MONO8;
  out_msg.image = rect_image;

  image_pub.publish(out_msg.toImageMsg());
}

void processDisparity(const cv::Mat &left_rect, const cv::Mat &right_rect,
                      const image_geometry::StereoCameraModel &model,
                      stereo_msgs::DisparityImage &disparity)
{
  // Fixed-point disparity is 16 times the true value: d = d_fp / 16.0 = x_l - x_r.
  static const int DPP = 16; // disparities per pixel
  static const double inv_dpp = 1.0 / DPP;

  cv::Mat disparity16_ = stereo_match(left_rect, right_rect);

  // Fill in DisparityImage image data, converting to 32-bit float
  sensor_msgs::Image &dimage = disparity.image;
  dimage.height = disparity16_.rows;
  dimage.width = disparity16_.cols;
  dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  dimage.step = dimage.width * sizeof(float);
  dimage.data.resize(dimage.step * dimage.height);
  cv::Mat_<float> dmat(dimage.height, dimage.width, (float *)&dimage.data[0], dimage.step);
  // We convert from fixed-point to float disparity and also adjust for any x-offset between
  // the principal points: d = d_fp*inv_dpp - (cx_l - cx_r)
  disparity16_.convertTo(dmat, dmat.type(), inv_dpp, -(model.left().cx() - model.right().cx()));
  ROS_ASSERT(dmat.data == &dimage.data[0]);
  /// @todo is_bigendian? :)

  // Stereo parameters
  disparity.f = model.right().fx();
  disparity.T = model.baseline();

  /// @todo Window of (potentially) valid disparities

  // Disparity search range
  disparity.min_disparity = _min_disparity;
  disparity.max_disparity = _min_disparity + _disparity_range - 1;
  disparity.delta_d = inv_dpp;
}

inline bool isValidPoint(const cv::Vec3f &pt)
{
  // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
  // and zero disparities (point mapped to infinity).
  return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);
}

void pointCloudCb(const stereo_msgs::DisparityImageConstPtr &msg_disp,
                  const sensor_msgs::ImageConstPtr &msg_img_l, const sensor_msgs::ImageConstPtr &msg_img_r,
                  const sensor_msgs::CameraInfoConstPtr &msg_info_l, const sensor_msgs::CameraInfoConstPtr &msg_info_r)
{
  image_geometry::StereoCameraModel model_;
  model_.fromCameraInfo(msg_info_l, msg_info_r);

  // Calculate point cloud
  const sensor_msgs::Image &dimage = msg_disp->image;
  const cv::Mat_<float> dmat(dimage.height, dimage.width, (float *)&dimage.data[0], dimage.step);
  cv::Mat_<cv::Vec3f> points_mat_; // scratch buffer
  model_.projectDisparityImageTo3d(dmat, points_mat_, true);

  PointCloudRGBNormal::Ptr ptCloudNormals(new PointCloudRGBNormal);
  cv_bridge::CvImagePtr input_image_left, input_image_right;
  input_image_left = cv_bridge::toCvCopy(msg_img_l, "mono8");
  _stereo_point_cloud_RGB = Mat2PCL(input_image_left->image, points_mat_, ptCloudNormals);

  cv::Mat_<cv::Vec3f> mat = points_mat_;

  // Fill in new PointCloud2 message (2D image-like layout)
  sensor_msgs::PointCloud2Ptr points_msg = boost::make_shared<sensor_msgs::PointCloud2>();
  points_msg->header = msg_disp->header;
  points_msg->height = mat.rows;
  points_msg->width = mat.cols;
  points_msg->is_bigendian = false;
  points_msg->is_dense = false; // there may be invalid points

  sensor_msgs::PointCloud2Modifier pcd_modifier(*points_msg);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*points_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*points_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*points_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*points_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*points_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*points_msg, "b");

  float bad_point = std::numeric_limits<float>::quiet_NaN();
  for (int v = 0; v < mat.rows; ++v)
  {
    for (int u = 0; u < mat.cols; ++u, ++iter_x, ++iter_y, ++iter_z)
    {
      if (isValidPoint(mat(v, u)))
      {
        // x,y,z
        *iter_x = mat(v, u)[0];
        *iter_y = mat(v, u)[1];
        *iter_z = mat(v, u)[2];
      }
      else
      {
        *iter_x = *iter_y = *iter_z = bad_point;
      }
    }
  }

  // Fill in color
  namespace enc = sensor_msgs::image_encodings;
  const std::string &encoding = msg_img_l->encoding;
  if (encoding == enc::MONO8)
  {
    const cv::Mat_<uint8_t> color(msg_img_l->height, msg_img_l->width,
                                  (uint8_t *)&msg_img_l->data[0],
                                  msg_img_l->step);
    for (int v = 0; v < mat.rows; ++v)
    {
      for (int u = 0; u < mat.cols; ++u, ++iter_r, ++iter_g, ++iter_b)
      {
        uint8_t g = color(v, u);
        *iter_r = *iter_g = *iter_b = g;
      }
    }
  }
  else if (encoding == enc::RGB8)
  {
    const cv::Mat_<cv::Vec3b> color(msg_img_l->height, msg_img_l->width,
                                    (cv::Vec3b *)&msg_img_l->data[0],
                                    msg_img_l->step);
    for (int v = 0; v < mat.rows; ++v)
    {
      for (int u = 0; u < mat.cols; ++u, ++iter_r, ++iter_g, ++iter_b)
      {
        const cv::Vec3b &rgb = color(v, u);
        *iter_r = rgb[0];
        *iter_g = rgb[1];
        *iter_b = rgb[2];
      }
    }
  }
  else if (encoding == enc::BGR8)
  {
    const cv::Mat_<cv::Vec3b> color(msg_img_l->height, msg_img_l->width,
                                    (cv::Vec3b *)&msg_img_l->data[0],
                                    msg_img_l->step);
    for (int v = 0; v < mat.rows; ++v)
    {
      for (int u = 0; u < mat.cols; ++u, ++iter_r, ++iter_g, ++iter_b)
      {
        const cv::Vec3b &bgr = color(v, u);
        *iter_r = bgr[2];
        *iter_g = bgr[1];
        *iter_b = bgr[0];
      }
    }
  }
  else
  {
    //NODELET_WARN_THROTTLE(30, "Could not fill color channel of the point cloud, "
    //                      "unsupported encoding '%s'", encoding.c_str());
  }

  _point_cloud_pub.publish(points_msg);
}

void imageCb(const sensor_msgs::ImageConstPtr &msg_left_image, const sensor_msgs::ImageConstPtr &msg_right_image, const sensor_msgs::CameraInfoConstPtr &msg_left_camera_info, const sensor_msgs::CameraInfoConstPtr &msg_right_camera_info)
{
  image_geometry::StereoCameraModel model_;
  // Update the camera model
  model_.fromCameraInfo(msg_left_camera_info, msg_right_camera_info);

  cv_bridge::CvImagePtr input_image_left, input_image_right;
  input_image_left = cv_bridge::toCvCopy(msg_left_image, "mono8");
  input_image_right = cv_bridge::toCvCopy(msg_right_image, "mono8");

  cv::Mat_<uint8_t> left_rect = rectify(input_image_left->image, msg_left_camera_info);
  cv::Mat_<uint8_t> right_rect = rectify(input_image_right->image, msg_right_camera_info);

  publish_image(_rect_l_pub, msg_left_image, left_rect);
  publish_image(_rect_r_pub, msg_right_image, right_rect);

  // Allocate new disparity image message
  stereo_msgs::DisparityImagePtr disp_msg = boost::make_shared<stereo_msgs::DisparityImage>();
  disp_msg->header = msg_left_camera_info->header;
  disp_msg->image.header = msg_left_camera_info->header;
  disp_msg->header.stamp = msg_left_camera_info->header.stamp;
  disp_msg->header.frame_id = _frame_id;

  disp_msg->valid_window.x_offset = 0;
  disp_msg->valid_window.y_offset = 0;
  disp_msg->valid_window.width = 0;
  disp_msg->valid_window.height = 0;

  // Perform block matching to find the disparities
  processDisparity(left_rect, right_rect, model_, *disp_msg);

  // Adjust for any x-offset between the principal points: d' = d - (cx_l - cx_r)
  double cx_l = model_.left().cx();
  double cx_r = model_.right().cx();
  if (cx_l != cx_r)
  {
    cv::Mat_<float> disp_image(disp_msg->image.height, disp_msg->image.width,
                               reinterpret_cast<float *>(&disp_msg->image.data[0]),
                               disp_msg->image.step);
    cv::subtract(disp_image, cv::Scalar(cx_l - cx_r), disp_image);
  }
  _disparity_pub.publish(disp_msg);

  pointCloudCb(disp_msg, msg_left_image, msg_right_image, msg_left_camera_info, msg_right_camera_info);

  _stereo_left = input_image_left->image;
  _stereo_right = input_image_right->image;
  _stereo_left_rect = left_rect;
  _stereo_right_rect = right_rect;
  cv_bridge::CvImagePtr disp_image = cv_bridge::toCvCopy(disp_msg->image, "32FC1");
  _stereo_disparity = disp_image->image;
  //TODO add global point cloud for saving
}

void parameterCallback(i3dr_stereo_camera::i3DR_DisparityConfig &config, uint32_t level)
{
  if (isInitParamConfig)
  {
    config.stereo_algorithm = _stereo_algorithm;
    config.correlation_window_size = _correlation_window_size;
    config.min_disparity = _min_disparity;
    config.disparity_range = _disparity_range;
    config.uniqueness_ratio = _uniqueness_ratio;
    config.texture_threshold = _texture_threshold;
    config.speckle_size = _speckle_size;
    config.speckle_range = _speckle_range;
    config.disp12MaxDiff = _disp12MaxDiff;
    config.p1 = _p1;
    config.p2 = _p2;
    config.interp = _interp;
    config.prefilter_cap = _preFilterCap;
    config.prefilter_size = _preFilterSize;
    isInitParamConfig = false;
  }
  else
  {
    config.prefilter_size |= 0x1; // must be odd
    _preFilterCap = config.prefilter_cap;
    _preFilterSize = config.prefilter_size;
    if (config.stereo_algorithm == CV_StereoBM || config.stereo_algorithm == CV_StereoSGBM)
    {
      config.correlation_window_size |= 0x1;                       //must be odd
      config.disparity_range = (config.disparity_range / 16) * 16; // must be multiple of 16
    }

    _stereo_algorithm = config.stereo_algorithm;
    if (_stereo_algorithm == CV_StereoBM)
    {
      matcher = block_matcher;
    }
    else if (_stereo_algorithm == CV_StereoSGBM)
    {
      matcher = sgbm_matcher;
    }
    else if (_stereo_algorithm == JR_StereoSGM)
    {
#ifdef ENABLE_I3DR_ALG
      matcher = jrsgm_matcher;
#else
      matcher = block_matcher;
      _stereo_algorithm = CV_StereoBM;
      config.stereo_algorithm = CV_StereoBM;
      ROS_ERROR("Not built to use I3DR algorithm. Resetting to block matcher.");
#endif
    }

    _correlation_window_size = config.correlation_window_size;
    _min_disparity = config.min_disparity;
    _disparity_range = config.disparity_range;
    _uniqueness_ratio = config.uniqueness_ratio;
    _texture_threshold = config.texture_threshold;
    _speckle_size = config.speckle_size;
    _speckle_range = config.speckle_range;
    _disp12MaxDiff = config.disp12MaxDiff;
    _p1 = config.p1;
    _p2 = config.p2;
    _interp = config.interp;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "generate_disparity");
  ros::NodeHandle nh;
  ros::NodeHandle p_nh("~");

  int stereo_algorithm, min_disparity, disparity_range, correlation_window_size, uniqueness_ratio, texture_threshold, speckle_size, speckle_range, disp12MaxDiff, preFilterCap, preFilterSize;

  float p1, p2;
  bool interp;
  std::string frame_id, left_camera_calibration_url, right_camera_calibration_url, jr_config_file;

  //Get parameters
  if (p_nh.getParam("stereo_algorithm", stereo_algorithm))
  {
    _stereo_algorithm = stereo_algorithm;
    ROS_INFO("stereo_algorithm: %d", _stereo_algorithm);
  }
  if (p_nh.getParam("min_disparity", min_disparity))
  {
    _min_disparity = min_disparity;
    ROS_INFO("min_disparity: %d", _min_disparity);
  }
  if (p_nh.getParam("disparity_range", disparity_range))
  {
    _disparity_range = disparity_range;
    ROS_INFO("disparity_range: %d", _disparity_range);
  }
  if (p_nh.getParam("correlation_window_size", correlation_window_size))
  {
    _correlation_window_size = correlation_window_size;
    ROS_INFO("correlation_window_size: %d", _correlation_window_size);
  }
  if (p_nh.getParam("uniqueness_ratio", uniqueness_ratio))
  {
    _uniqueness_ratio = uniqueness_ratio;
    ROS_INFO("uniqueness_ratio: %d", _uniqueness_ratio);
  }
  if (p_nh.getParam("texture_threshold", texture_threshold))
  {
    _texture_threshold = texture_threshold;
    ROS_INFO("texture_threshold: %d", _texture_threshold);
  }
  if (p_nh.getParam("speckle_size", speckle_size))
  {
    _speckle_size = speckle_size;
    ROS_INFO("speckle_size: %d", _speckle_size);
  }
  if (p_nh.getParam("speckle_range", speckle_range))
  {
    _speckle_range = speckle_range;
    ROS_INFO("speckle_range: %d", _speckle_range);
  }
  if (p_nh.getParam("disp12MaxDiff", disp12MaxDiff))
  {
    _disp12MaxDiff = disp12MaxDiff;
    ROS_INFO("disp12MaxDiff: %d", _disp12MaxDiff);
  }
  if (p_nh.getParam("p1", p1))
  {
    _p1 = p1;
    ROS_INFO("p1: %f", _p1);
  }
  if (p_nh.getParam("p2", p2))
  {
    _p2 = p2;
    ROS_INFO("p2: %f", _p2);
  }
  if (p_nh.getParam("pre_filter_cap", preFilterCap))
  {
    _preFilterCap = preFilterCap;
    ROS_INFO("pre_filter_cap %d", _preFilterCap);
  }
  if (p_nh.getParam("pre_filter_size", preFilterSize))
  {
    _preFilterSize = preFilterSize;
    ROS_INFO("pre_filter_size %d", _preFilterSize);
  }
  if (p_nh.getParam("frame_id", frame_id))
  {
    _frame_id = frame_id;
    ROS_INFO("frame_id: %s", _frame_id.c_str());
  }
  if (p_nh.getParam("jr_config_file", jr_config_file))
  {
    _jr_config_file = jr_config_file;
    ROS_INFO("jr_config_file: %s", _jr_config_file.c_str());
  }
  if (p_nh.getParam("interp", interp))
  {
    _interp = interp;
    ROS_INFO("interp: %s", interp ? "true" : "false");
  }

  std::string empty_str = " ";

  block_matcher = new MatcherOpenCVBlock(empty_str);
  sgbm_matcher = new MatcherOpenCVSGBM(empty_str);
#ifdef ENABLE_I3DR_ALG
  jrsgm_matcher = new MatcherJRSGM(_jr_config_file);
#endif

  if (_stereo_algorithm == CV_StereoBM)
  {
    matcher = block_matcher;
  }
  else if (_stereo_algorithm == CV_StereoSGBM)
  {
    matcher = sgbm_matcher;
  }
  else if (_stereo_algorithm == JR_StereoSGM)
  {
#ifdef ENABLE_I3DR_ALG
    matcher = jrsgm_matcher;
#else
    matcher = block_matcher;
    _stereo_algorithm = CV_StereoBM;
    ROS_ERROR("Not built to use I3DR algorithm. Resetting to block matcher.");
#endif
  }

  std::string ns = ros::this_node::getNamespace();

  // Dynamic parameters
  dynamic_reconfigure::Server<i3dr_stereo_camera::i3DR_DisparityConfig> server;
  dynamic_reconfigure::Server<i3dr_stereo_camera::i3DR_DisparityConfig>::CallbackType f;
  f = boost::bind(&parameterCallback, _1, _2);
  server.setCallback(f);

  // Publishers creation
  _disparity_pub = nh.advertise<stereo_msgs::DisparityImage>(ns + "/disparity", 1, true);
  _rect_l_pub = nh.advertise<sensor_msgs::Image>(ns + "/left/image_rect", 1, true);
  _rect_r_pub = nh.advertise<sensor_msgs::Image>(ns + "/right/image_rect", 1, true);
  _point_cloud_pub = nh.advertise<PointCloudRGB>(ns + "/points2", 1);
  _point_cloud_normal_pub = nh.advertise<PointCloudRGBNormal>(ns + "/points2_normal", 1);

  // Start services
  ros::ServiceServer srv_save_stereo = nh.advertiseService("save_stereo", save_stereo);

  // Subscribers creation.
  message_filters::Subscriber<sensor_msgs::Image> sub_image_l(nh, ns + "/left/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> sub_image_r(nh, ns + "/right/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_l(nh, ns + "/left/camera_info", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_r(nh, ns + "/right/camera_info", 1);

  // Message filter creation.
  message_filters::Synchronizer<policy_t> sync(policy_t(10), sub_image_l, sub_image_r, sub_camera_info_l, sub_camera_info_r);
  sync.registerCallback(boost::bind(&imageCb, _1, _2, _3, _4));

  ros::spin();
}
