#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_geometry/stereo_camera_model.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ximgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

ros::Publisher point_cloud_pub, point_cloud_normal_pub;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudRGBNormal;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

typedef message_filters::sync_policies::ApproximateTime<
    stereo_msgs::DisparityImage, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>
    policy_t;

cv::Mat generateQ(double cx, double cy, double cx1, double focal_length, double baseline){
    cv::Mat Q(cv::Size(4,4),CV_64FC1,cv::Scalar(0));
    Q.at<double>(0,0)=1.0;
    Q.at<double>(0,1)=0.0;
    Q.at<double>(0,2)=0.0;
    Q.at<double>(0,3)=-cx; //cx
    Q.at<double>(1,0)=0.0;
    Q.at<double>(1,1)=1.0;
    Q.at<double>(1,2)=0.0;
    Q.at<double>(1,3)=-cy;  //cy
    Q.at<double>(2,0)=0.0;
    Q.at<double>(2,1)=0.0;
    Q.at<double>(2,2)=0.0;
    Q.at<double>(2,3)=focal_length;  //Focal
    Q.at<double>(3,0)=0.0;
    Q.at<double>(3,1)=0.0;
    Q.at<double>(3,2)=1.0/baseline;    //-1.0/BaseLine
    Q.at<double>(3,3)=cx-cx1/baseline;    //cx - cx'/Baseline
    return Q;
}

cv::Mat reproject(cv::Mat dimage, const sensor_msgs::CameraInfoConstPtr &camera_info_l, const sensor_msgs::CameraInfoConstPtr &camera_info_r){
  
  double cx = camera_info_l->K[2];
  double cx1 = camera_info_r->K[2];
  double cy = camera_info_l->K[5];
  double f = camera_info_l->K[0];
  double T = camera_info_r->P[3];
  cv::Mat Q = generateQ(cx,cy,cx1,f,T);

  cv::Mat xyz;
  cv::reprojectImageTo3D(dimage, xyz, Q, true);
  return(xyz);
}

//Convert disparity image from opencv Mat to PCL Point Cloud XYZRGB
PointCloudRGB::Ptr Mat2PCL(cv::Mat image, cv::Mat coords, PointCloudRGBNormal::Ptr normals){

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

    for (int i = 1; i < coords.rows-1; i++) {
        float *reconst_ptr = coords.ptr<float>(i);
        uchar *rgb_ptr = image.ptr<uchar>(i);

        if(!rgb_ptr || !reconst_ptr) return(ptCloudTemp);

        for (int j = 1; j < coords.cols-1; j++) {
            if (rgb_ptr[j] == 0) continue;

            point.x = reconst_ptr[3 * j];
            point.y = reconst_ptr[3 * j + 1];
            point.z = reconst_ptr[3 * j + 2];

            if(abs(point.x) > 10) continue;
            if(abs(point.y) > 10) continue;
            if(abs(point.z) > 10) continue;
            col = rgb_ptr[j];

            rgb = ((int)col) << 16 | ((int)col) << 8 | ((int)col);
            point.rgb = *reinterpret_cast<float *>(&rgb);

            //normals
            float dzdx = (coords.at<float>(i+1, j) - coords.at<float>(i-1, j)) / 2.0;
            float dzdy = (coords.at<float>(i, j+1) - coords.at<float>(i, j-1)) / 2.0;

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
    pcl::copyPointCloud(*ptCloudNormals,*normals);
    return(ptCloudTemp);
}

/*
//Convert disparity image from opencv Mat to PCL Point Cloud XYZRGB
PointCloudRGB::Ptr Mat2PCL(cv::Mat limage, cv::Mat xyz, PointCloudRGBNormal::Ptr normal){

    PointCloudRGB::Ptr ptCloudTemp(new PointCloudRGB);
    PointCloudRGBNormal::Ptr ptCloudNormal(new PointCloudRGBNormal);

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
    ptCloudNormal->points.push_back(pNormal);

    for (int i = 1; i < xyz.rows; i++) {
        float *reconst_ptr = xyz.ptr<float>(i);
        uchar *rgb_ptr = limage.ptr<uchar>(i);

        if(!rgb_ptr || !reconst_ptr) return(ptCloudTemp);

        for (int j = 1; j < xyz.cols; j++) {
            if (rgb_ptr[j] == 0) continue;

            point.x = reconst_ptr[3 * j];
            point.y = reconst_ptr[3 * j + 1];
            point.z = reconst_ptr[3 * j + 2];

            if(abs(point.x) > 10) continue;
            if(abs(point.y) > 10) continue;
            if(abs(point.z) > 10) continue;
            col = rgb_ptr[j];

            rgb = ((int)col) << 16 | ((int)col) << 8 | ((int)col);
            point.rgb = *reinterpret_cast<float *>(&rgb);

            //normal
            float dzdx = (xyz.at<float>(i+1, j) - xyz.at<float>(i-1, j)) / 2.0;
            float dzdy = (xyz.at<float>(i, j+1) - xyz.at<float>(i, j-1)) / 2.0;

            cv::Vec3f d(-dzdx, -dzdy, 1.0f);

            cv::Vec3f n = normalize(d);

            pNormal.x = point.x;
            pNormal.y = point.y;
            pNormal.z = point.z;

            pNormal.rgb = point.rgb;

            pNormal.normal_x = n[0];
            pNormal.normal_y = n[1];
            pNormal.normal_z = n[2];

            ptCloudNormal->points.push_back(pNormal);

            ptCloudTemp->points.push_back(point);
        }
    }
    pcl::copyPointCloud(*ptCloudNormal,*normal);
    return(ptCloudTemp);
}
*/

inline bool isValidPoint(const cv::Vec3f& pt)
{
  // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
  // and zero disparities (point mapped to infinity).
  return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);
}

void imageCb(const stereo_msgs::DisparityImageConstPtr& msg_disp, 
  const sensor_msgs::ImageConstPtr &msg_img_l, const sensor_msgs::ImageConstPtr &msg_img_r,
  const sensor_msgs::CameraInfoConstPtr &msg_info_l, const sensor_msgs::CameraInfoConstPtr &msg_info_r)
{
  ROS_INFO("disparity received");
  image_geometry::StereoCameraModel model_;
  model_.fromCameraInfo(msg_info_l, msg_info_r);

  // Calculate point cloud
  const sensor_msgs::Image& dimage = msg_disp->image;
  const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
  cv::Mat_<cv::Vec3f> points_mat_; // scratch buffer
  model_.projectDisparityImageTo3d(dmat, points_mat_, true);
  //cv::reprojectImageTo3D(dmat, points_mat_,Q, true);
  //points_mat_ = reproject(dmat,msg_info_l,msg_info_r);
  cv::Mat_<cv::Vec3f> mat = points_mat_;

  // Fill in new PointCloud2 message (2D image-like layout)
  sensor_msgs::PointCloud2Ptr points_msg = boost::make_shared<sensor_msgs::PointCloud2>();
  points_msg->header = msg_disp->header;
  points_msg->height = mat.rows;
  points_msg->width  = mat.cols;
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

  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  for (int v = 0; v < mat.rows; ++v)
  {
    for (int u = 0; u < mat.cols; ++u, ++iter_x, ++iter_y, ++iter_z)
    {
      if (isValidPoint(mat(v,u)))
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
  const std::string& encoding = msg_img_l->encoding;
  if (encoding == enc::MONO8)
  {
    const cv::Mat_<uint8_t> color(msg_img_l->height, msg_img_l->width,
                                  (uint8_t*)&msg_img_l->data[0],
                                  msg_img_l->step);
    for (int v = 0; v < mat.rows; ++v)
    {
      for (int u = 0; u < mat.cols; ++u, ++iter_r, ++iter_g, ++iter_b)
      {
        uint8_t g = color(v,u);
        *iter_r = *iter_g = *iter_b = g;
      }
    }
  }
  else if (encoding == enc::RGB8)
  {
    const cv::Mat_<cv::Vec3b> color(msg_img_l->height, msg_img_l->width,
                                    (cv::Vec3b*)&msg_img_l->data[0],
                                    msg_img_l->step);
    for (int v = 0; v < mat.rows; ++v)
    {
      for (int u = 0; u < mat.cols; ++u, ++iter_r, ++iter_g, ++iter_b)
      {
        const cv::Vec3b& rgb = color(v,u);
        *iter_r = rgb[0];
        *iter_g = rgb[1];
        *iter_b = rgb[2];
      }
    }
  }
  else if (encoding == enc::BGR8)
  {
    const cv::Mat_<cv::Vec3b> color(msg_img_l->height, msg_img_l->width,
                                    (cv::Vec3b*)&msg_img_l->data[0],
                                    msg_img_l->step);
    for (int v = 0; v < mat.rows; ++v)
    {
      for (int u = 0; u < mat.cols; ++u, ++iter_r, ++iter_g, ++iter_b)
      {
        const cv::Vec3b& bgr = color(v,u);
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

  point_cloud_pub.publish(points_msg);
}

//Get ROS disparity msg from stereo camera and send to processing
void disparityCallback(const stereo_msgs::DisparityImageConstPtr& msg_disp, 
  const sensor_msgs::ImageConstPtr &msg_img_l, const sensor_msgs::ImageConstPtr &msg_img_r,
  const sensor_msgs::CameraInfoConstPtr &msg_info_l, const sensor_msgs::CameraInfoConstPtr &msg_info_r)
{
  cv_bridge::CvImagePtr input_disparity, input_image_l, input_image_r;
  input_image_l = cv_bridge::toCvCopy(msg_img_l);
  input_image_r = cv_bridge::toCvCopy(msg_img_r);

  PointCloudRGBNormal::Ptr point_cloud_normal (new PointCloudRGBNormal);
  PointCloudRGB::Ptr point_cloud (new PointCloudRGB);

  const sensor_msgs::Image& dimage_msg = msg_disp->image;
  cv_bridge::CvImagePtr disparity_image;
  disparity_image = cv_bridge::toCvCopy(dimage_msg);
  cv::Mat dimage = disparity_image->image;

  cv::Mat xyz = reproject(dimage, msg_info_l,msg_info_r);
  point_cloud = Mat2PCL(input_image_l->image,xyz,point_cloud_normal);

  //point_cloud->header.frame_id = msg_disp->header.frame_id;
  //point_cloud_normal->header.frame_id = msg_disp->header.frame_id;
  point_cloud->header.frame_id = "deimos_depth_optical_frame";
  point_cloud_normal->header.frame_id = "deimos_depth_optical_frame";
  pcl_conversions::toPCL(msg_disp->header.stamp, point_cloud->header.stamp);
  pcl_conversions::toPCL(msg_disp->header.stamp, point_cloud_normal->header.stamp);
  
  sensor_msgs::PointCloud2 pointcloud_msg,pointcloud_normal_msg;
  pcl::toROSMsg(*point_cloud,pointcloud_msg);
  pcl::toROSMsg(*point_cloud_normal,pointcloud_normal_msg);

  point_cloud_pub.publish(pointcloud_msg);
  point_cloud_normal_pub.publish(pointcloud_normal_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "generate_point_cloud");
  ros::NodeHandle nh;
  ros::NodeHandle pnode("~");

  std::string ns = ros::this_node::getNamespace();

  // Subscribers creation
  message_filters::Subscriber<stereo_msgs::DisparityImage> disparity_sub(nh, ns+"/disparity", 1);
  message_filters::Subscriber<sensor_msgs::Image> image_l_sub(nh, ns+"/left/image_rect", 1);
  message_filters::Subscriber<sensor_msgs::Image> image_r_sub(nh, ns+"/right/image_rect", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_l_sub(nh, ns+"/left/camera_info", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_r_sub(nh, ns+"/right/camera_info", 1);

  int queue_size;
  pnode.param("queue_size", queue_size, 5);

  // Message filter creation.
  message_filters::Synchronizer<policy_t> sync(policy_t(queue_size), disparity_sub, image_l_sub, image_r_sub, camera_info_l_sub, camera_info_r_sub);
  //sync.registerCallback(boost::bind(&disparityCallback, _1, _2, _3, _4, _5));
  sync.registerCallback(boost::bind(&imageCb, _1, _2, _3, _4, _5));

  point_cloud_pub = nh.advertise<PointCloudRGB>(ns+"/i3dr_points2", 1);
  point_cloud_normal_pub = nh.advertise<PointCloudRGBNormal>(ns+"/i3dr_points2_normal", 1);

  ros::spin();
}