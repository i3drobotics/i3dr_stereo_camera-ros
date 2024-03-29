#include <ros/ros.h>
#include <ros/console.h>
#include <ros/param.h>
#include <ros/package.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <opencv2/ximgproc.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

#include "boost/date_time/posix_time/posix_time.hpp"

#include "StereoGUI/StereoGUI.h"
#include <QApplication>
#include <QTimer>
#include <QGraphicsScene>

using namespace cv;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>
    policy_image;

typedef message_filters::sync_policies::ApproximateTime<
    stereo_msgs::DisparityImage, sensor_msgs::Image, sensor_msgs::PointCloud2>
    policy_depth;

cv::Mat left_image_, right_image_, right_rect_image_, left_rect_image_;
cv::Mat disp_image_, depth_image_;
PointCloudT::Ptr _point_cloud;
StereoGUI *stereoGUI;
QTimer *rosUpdateTimer;

void updateGUILastUpdateTime(ros::Time time){
    boost::posix_time::ptime my_posix_time = time.toBoost();
    std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);
    std::string text = iso_time_str;
    stereoGUI->updateTimeText(text);
}

void updateGUIImage(const sensor_msgs::ImageConstPtr &msg_image, QGraphicsScene *gs){
    cv::Mat image;
    image = cv_bridge::toCvCopy(msg_image)->image;
    cv::Mat image_bgr;
    cvtColor(image, image_bgr, COLOR_GRAY2BGR);
    stereoGUI->updateImage(image_bgr,gs);
}

void updateGUIDepthImage(const sensor_msgs::ImageConstPtr &msg_image, QGraphicsScene *gs){
    cv::Mat image;
    image = cv_bridge::toCvCopy(msg_image,sensor_msgs::image_encodings::TYPE_32FC1)->image;
    cv::normalize(image, image, 1, 0, NORM_MINMAX);
    stereoGUI->updateDepthImage(image,gs);
}

void imageCb(const sensor_msgs::ImageConstPtr &msg_left_image, const sensor_msgs::ImageConstPtr &msg_right_image, const sensor_msgs::CameraInfoConstPtr &msg_left_camera_info, const sensor_msgs::CameraInfoConstPtr &msg_right_camera_info)
{
    updateGUIImage(msg_left_image,stereoGUI->getLeftGS());
    updateGUIImage(msg_right_image,stereoGUI->getRightGS());
    updateGUILastUpdateTime(ros::Time::now());
}

void imageRectCb(const sensor_msgs::ImageConstPtr &msg_left_image, const sensor_msgs::ImageConstPtr &msg_right_image, const sensor_msgs::CameraInfoConstPtr &msg_left_camera_info, const sensor_msgs::CameraInfoConstPtr &msg_right_camera_info)
{
    updateGUILastUpdateTime(ros::Time::now());
}

void depthCb(const stereo_msgs::DisparityImageConstPtr &msg_disp, const sensor_msgs::ImageConstPtr &msg_depth, const sensor_msgs::PointCloud2ConstPtr &msg_points)
{
    updateGUIDepthImage(msg_depth,stereoGUI->getDepthGS());

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg_points, pcl_pc2);
    PointCloudT::Ptr _point_cloud = PointCloudT::Ptr(new PointCloudT);
    pcl::fromPCLPointCloud2(pcl_pc2, *_point_cloud);
    stereoGUI->updatePoints(_point_cloud);

    updateGUILastUpdateTime(ros::Time::now());
}

void updateROS(){
    if (ros::ok()){
        ros::spinOnce();
    } else {
        rosUpdateTimer->stop();
        stereoGUI->close();
    }
}

int main(int argc, char **argv)
{
    // Creates an instance of QApplication
    QApplication app(argc, argv);
    
    // Create ros node instance
    ros::init(argc, argv, "stereo_gui");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    std::string ns = ros::this_node::getNamespace();

    // Subscribers creation.
    message_filters::Subscriber<sensor_msgs::Image> sub_image_l(nh, ns + "/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_image_r(nh, ns + "/right/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_l(nh, ns + "/left/camera_info", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_r(nh, ns + "/right/camera_info", 1);

    message_filters::Subscriber<sensor_msgs::Image> sub_image_rect_l(nh, ns + "/left/image_rect", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_image_rect_r(nh, ns + "/right/image_rect", 1);

    message_filters::Subscriber<stereo_msgs::DisparityImage> sub_disparity(nh, ns + "/disparity", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth(nh, ns + "/depth", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_points(nh, ns + "/points2", 1);

    // Message filter creation.
    message_filters::Synchronizer<policy_image> sync_image(policy_image(10), sub_image_l, sub_image_r, sub_camera_info_l, sub_camera_info_r);
    sync_image.registerCallback(boost::bind(&imageCb, _1, _2, _3, _4));

    message_filters::Synchronizer<policy_image> sync_image_rect(policy_image(10), sub_image_rect_l, sub_image_rect_r, sub_camera_info_l, sub_camera_info_r);
    sync_image_rect.registerCallback(boost::bind(&imageRectCb, _1, _2, _3, _4));

    message_filters::Synchronizer<policy_depth> sync_depth(policy_depth(10), sub_disparity, sub_depth, sub_points);
    sync_depth.registerCallback(boost::bind(&depthCb, _1, _2, _3));

    stereoGUI = new StereoGUI();
    stereoGUI->show(); // Show main window

    rosUpdateTimer = new QTimer();
    QObject::connect(rosUpdateTimer, &QTimer::timeout, updateROS);
    rosUpdateTimer->start();

    int exit_code = app.exec();

    rosUpdateTimer->stop();
    return exit_code;
}