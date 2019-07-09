#define REVERSE_DISPARITY_EN 0

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/param.h>

#include <boost/bind.hpp>

#include "std_msgs/String.h"
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

#if REVERSE_DISPARITY_EN
    #include <opencv2/ximgproc.hpp>
#endif

#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>

#include <dynamic_reconfigure/server.h>
#include <i3dr_stereo_camera/i3DR_DisparityConfig.h>

#include <boost/filesystem.hpp>
#include <matcherJrsgm.h>

#include <string>

using namespace cv;

int CV_StereoBM = 0;
int CV_StereoSGBM = 1;
int JR_StereoSGBM = 2;
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
float _p1 = 200;
float _p2 = 400;
cv::Mat _Kl, _Dl, _Rl, _Pl;
cv::Mat _Kr, _Dr, _Rr, _Pr;

typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>
    policy_t;

void cameraInfo_to_KDRP(const sensor_msgs::CameraInfoConstPtr &msg_camera_info, cv::Mat &K, cv::Mat &D, cv::Mat &R, cv::Mat &P){
    K = cv::Mat(3, 3, CV_64FC1, (void *)msg_camera_info->K.data());
    D = cv::Mat(1, 5, CV_64FC1, (void *)msg_camera_info->D.data());
    R = cv::Mat(3, 3, CV_64FC1, (void *)msg_camera_info->R.data());
    P = cv::Mat(3, 4, CV_64FC1, (void *)msg_camera_info->P.data());
}

//Calculate disparity using left and right images
Mat stereo_match(Mat left_image, Mat right_image, int algorithm, int min_disparity, int disparity_range, int correlation_window_size, int uniqueness_ratio, int texture_threshold, int speckleSize, int speckelRange, float p1, float p2)
{
    cv::Mat disp, disparity_rl, disparity_filter;
    cv::Size image_size = cv::Size(left_image.size().width, left_image.size().height);
    // Setup for 16-bit disparity
    cv::Mat(image_size, CV_16S).copyTo(disp);
    cv::Mat(image_size, CV_16S).copyTo(disparity_rl);
    cv::Mat(image_size, CV_16S).copyTo(disparity_filter);

    if (disparity_range < 1 || disparity_range % 16 != 0)
    {
        ROS_ERROR("disparity_range must be a positive integer divisible by 16");
        return disp;
    }

    //disparity_range = disparity_range > 0 ? disparity_range : ((left_image.size().width / 8) + 15) & -16;

    if (algorithm == CV_StereoBM)
    {
        cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(64, 9);

        bm->setPreFilterCap(31);
        bm->setPreFilterSize(15);
        bm->setPreFilterType(1);
        bm->setBlockSize(correlation_window_size > 0 ? correlation_window_size : 9);
        bm->setMinDisparity(min_disparity);
        bm->setNumDisparities(disparity_range);
        bm->setTextureThreshold(texture_threshold);
        bm->setUniquenessRatio(uniqueness_ratio);
        bm->setSpeckleWindowSize(speckleSize);
        bm->setSpeckleRange(speckelRange);
        bm->setDisp12MaxDiff(-1);

        bm->compute(left_image, right_image, disp);

        #if REVERSE_DISPARITY_EN
            auto right_matcher = cv::ximgproc::createRightMatcher(bm);
            right_matcher->compute(right_image, left_image, disparity_rl);

            double wls_lambda = 8000;
            double wls_sigma = 1.5;
            cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter(bm);
            wls_filter->setLambda(wls_lambda);
            wls_filter->setSigmaColor(wls_sigma);
            wls_filter->filter(disp, left_image, disparity_filter, disparity_rl);
        #endif
    }
    else if (algorithm == CV_StereoSGBM)
    {
        cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(64, 9);

        sgbm->setPreFilterCap(31);
        //sgbm->setP1(1);
        //sgbm->setP2(1);
        sgbm->setBlockSize(correlation_window_size > 0 ? correlation_window_size : 9);
        sgbm->setMinDisparity(min_disparity);
        sgbm->setNumDisparities(disparity_range);
        sgbm->setUniquenessRatio(uniqueness_ratio);
        sgbm->setSpeckleWindowSize(speckleSize);
        sgbm->setSpeckleRange(speckelRange);
        sgbm->setDisp12MaxDiff(-1);
        sgbm->setP1(p1);
        sgbm->setP2(p2);

        sgbm->compute(left_image, right_image, disp);

        #if REVERSE_DISPARITY_EN
            auto right_matcher = cv::ximgproc::createRightMatcher(sgbm);
            right_matcher->compute(right_image, left_image, disparity_rl);

            double wls_lambda = 8000;
            double wls_sigma = 1.5;
            cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter(sgbm);
            wls_filter->setLambda(wls_lambda);
            wls_filter->setSigmaColor(wls_sigma);
            wls_filter->filter(disp, left_image, disparity_filter, disparity_rl);
        #endif
    }
    else if (algorithm == JR_StereoSGBM){
        MatcherJrSGM* matcher = new MatcherJrSGM;
        matcher->setDisparityRange(disparity_range);
        matcher->setDisparityShift(min_disparity);
        matcher->setMatchCosts(p1,p2);
        matcher->setWindowSize(correlation_window_size > 0 ? correlation_window_size : 9);
        
        matcher->compute(left_image, right_image, disp);
    }
    //disp.convertTo(disp, CV_32F);
    //disparity_filter.convertTo(disparity_filter, CV_32F);
    return disp;
}

void publish_disparity(cv::Mat disparity, int min_disparity, int disparity_range,  const sensor_msgs::CameraInfoConstPtr &msg_left_camera_info, const sensor_msgs::CameraInfoConstPtr &msg_right_camera_info)
{
    stereo_msgs::DisparityImage disp_msg;
    disp_msg.min_disparity = min_disparity;
    disp_msg.max_disparity = min_disparity + disparity_range - 1;

    cv::Mat Kl,Dl,Rl,Pl;
    cv::Mat Kr,Dr,Rr,Pr;
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

    double cxR = Kr.at<double>(0,2);
    double cxL = Kl.at<double>(0,2);
    double f = Kl.at<double>(0,0);
    double T = Pr.at<double>(0,3) / Kr.at<double>(0,0); //baseline = P14 / f

    disparity.convertTo(dmat, dmat.type(), inv_dpp, -(cxL - cxR));
    ROS_ASSERT(dmat.data == &dimage.data[0]);

    disp_msg.delta_d = inv_dpp;

    disp_msg.f = f;
    disp_msg.T = T;

    _disparity_pub.publish(disp_msg);
}

void parameterCallback(i3dr_stereo_camera::i3DR_DisparityConfig &config, uint32_t level)
{
    config.prefilter_size |= 0x1;                                // must be odd
    config.correlation_window_size |= 0x1;                       //must be odd
    config.disparity_range = (config.disparity_range / 16) * 16; // must be multiple of 16

    _stereo_algorithm = config.stereo_algorithm;
    _correlation_window_size = config.correlation_window_size;
    _min_disparity = config.min_disparity;
    _disparity_range = config.disparity_range;
    _uniqueness_ratio = config.uniqueness_ratio;
    _texture_threshold = config.texture_threshold;
    _speckle_size = config.speckle_size;
    _speckle_range = config.speckle_range;
    _p1 = config.P1;
    _p2 = config.P2;
}

cv::Mat rectify(cv::Mat image, const sensor_msgs::CameraInfoConstPtr &msg_camera_info)
{
    cv::Mat K,D,R,P;
    cameraInfo_to_KDRP(msg_camera_info, K, D, R, P);
    
    cv::Size resol = cv::Size(image.size().width, image.size().height);

    cv::Mat full_map1, full_map2;

    cv::initUndistortRectifyMap(K, D, R, P, resol,
                                CV_32FC1, full_map1, full_map2);
                                
    cv::Mat image_rect;
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

void stereoImageCallback(const sensor_msgs::ImageConstPtr &msg_left_image, const sensor_msgs::ImageConstPtr &msg_right_image, const sensor_msgs::CameraInfoConstPtr &msg_left_camera_info, const sensor_msgs::CameraInfoConstPtr &msg_right_camera_info)
{
    try
    {
        cv_bridge::CvImagePtr input_image_left, input_image_right;
        input_image_left = cv_bridge::toCvCopy(msg_left_image, "mono8");
        input_image_right = cv_bridge::toCvCopy(msg_right_image, "mono8");
    
        cv::Mat left_rect = rectify(input_image_left->image, msg_left_camera_info);
        cv::Mat right_rect = rectify(input_image_right->image, msg_right_camera_info);

        publish_image(_rect_l_pub, msg_left_image, left_rect);
        publish_image(_rect_r_pub, msg_right_image, right_rect);

        Mat disp = stereo_match(left_rect, right_rect, _stereo_algorithm, _min_disparity, _disparity_range, _correlation_window_size, _uniqueness_ratio, _texture_threshold, _speckle_size, _speckle_range,_p1,_p2);
        publish_disparity(disp, _min_disparity, _disparity_range, msg_left_camera_info, msg_right_camera_info);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("exception %s",e.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "generate_disparity");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    int stereo_algorithm, min_disparity, disparity_range, correlation_window_size, uniqueness_ratio, texture_threshold, speckle_size, speckle_range;
    std::string frame_id, left_camera_calibration_url, right_camera_calibration_url;

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
    if (p_nh.getParam("frame_id", frame_id))
    {
        _frame_id = frame_id;
        ROS_INFO("frame_id: %s", _frame_id.c_str());
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

    // Subscribers creation.
    message_filters::Subscriber<sensor_msgs::Image> sub_image_l(nh, ns + "/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_image_r(nh, ns + "/right/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_l(nh, ns + "/left/camera_info", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_r(nh, ns + "/right/camera_info", 1);

    // Message filter creation.
    message_filters::Synchronizer<policy_t> sync(policy_t(10), sub_image_l, sub_image_r, sub_camera_info_l, sub_camera_info_r);
    sync.registerCallback(boost::bind(&stereoImageCallback, _1, _2, _3, _4));

    ros::spin();
}
