#ifndef STEREOGUI_H
#define STEREOGUI_H

// QT
#include <QMainWindow>
#include <QGraphicsScene>

// VTK
#include <vtkRenderWindow.h>
#include <QVTKWidget.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui {
class StereoGUI;
}

class StereoGUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit StereoGUI(QWidget *parent = 0);
    ~StereoGUI();
    void updateTimeText(std::string text);
    void updateImage(cv::Mat image, QGraphicsScene *gs);
    void updateDepthImage(cv::Mat image, QGraphicsScene *gs);
    void updatePoints(PointCloudT::Ptr points);
    QGraphicsScene *getLeftGS(){return gs_left;};
    QGraphicsScene *getRightGS(){return gs_right;};
    QGraphicsScene *getDepthGS(){return gs_depth;};

private:
    Ui::StereoGUI *ui;
    QGraphicsScene *gs_left;
    QGraphicsScene *gs_right;
    QGraphicsScene *gs_depth;
    int image_height;
    int image_width;
    PointCloudT::Ptr cloud;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer;
    QImage Mat2QImage(cv::Mat const& src);
    QImage DepthMat2QImage(const cv::Mat_<double> &src);
    cv::Mat QImage2Mat(QImage const& src);
};

#endif // STEREOGUI_H