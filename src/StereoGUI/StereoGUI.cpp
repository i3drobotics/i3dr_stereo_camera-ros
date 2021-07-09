#include "StereoGUI/StereoGUI.h"
#include "ui_StereoGUI.h"

StereoGUI::StereoGUI(QWidget *parent) : QMainWindow(parent),
                                        ui(new Ui::StereoGUI)
{
    ui->setupUi(this);
    gs_left = new QGraphicsScene(this);
    gs_right = new QGraphicsScene(this);
    gs_depth = new QGraphicsScene(this);
    ui->gvCamLeft->setScene(gs_left);
    ui->gvCamRight->setScene(gs_right);
    ui->gvDispLeft->setScene(gs_left);
    ui->gvDispRight->setScene(gs_depth);
    image_height = ui->gvCamLeft->height();
    image_width = ui->gvCamLeft->width();

    // initalise point cloud viewer
    cloud.reset(new PointCloudT);
    cloud_viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    ui->qvtkCloud->SetRenderWindow(cloud_viewer->getRenderWindow());
    cloud_viewer->setupInteractor(ui->qvtkCloud->GetInteractor(),
                                  ui->qvtkCloud->GetRenderWindow());
    ui->qvtkCloud->update();
}

QImage StereoGUI::Mat2QImage(cv::Mat const &src)
{
    cv::Mat temp;                    // make the same cv::Mat
    cvtColor(src, temp, cv::COLOR_BGR2RGB); // cvtColor Makes a copt, that what i need
    QImage dest((const uchar *)temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
    dest.bits(); // enforce deep copy, see documentation
    // of QImage::QImage ( const uchar * data, int width, int height, Format format )
    return dest;
}

QImage StereoGUI::DepthMat2QImage(const cv::Mat_<double> &src)
{
    double scale = 255.0;
    QImage dest(src.cols, src.rows, QImage::Format_RGB32);
    for (int y = 0; y < src.rows; ++y)
    {
        const double *srcrow = src[y];
        QRgb *destrow = (QRgb *)dest.scanLine(y);
        for (int x = 0; x < src.cols; ++x)
        {
            unsigned int color = srcrow[x] * scale;
            destrow[x] = qRgb(color, color, color);
        }
    }
    return dest;
}

cv::Mat StereoGUI::QImage2Mat(QImage const &src)
{
    cv::Mat tmp(src.height(), src.width(), CV_8UC3, (uchar *)src.bits(), src.bytesPerLine());
    cv::Mat result; // deep copy just in case (my lack of knowledge with open cv)
    cvtColor(tmp, result, cv::COLOR_BGR2RGB);
    return result;
}

void StereoGUI::updateTimeText(std::string text)
{
    QString qstr = QString::fromStdString(text);
    ui->lblLastUpdateTime->setText(qstr);
}

void StereoGUI::updateDepthImage(cv::Mat image, QGraphicsScene *gs)
{
    cv::resize(image, image, cv::Size(image_width, image_height));
    QImage qimage = DepthMat2QImage(image);
    gs->addPixmap(QPixmap::fromImage(qimage));
}

void StereoGUI::updateImage(cv::Mat image, QGraphicsScene *gs)
{
    cv::resize(image, image, cv::Size(image_width, image_height));
    QImage qimage = Mat2QImage(image);
    gs->addPixmap(QPixmap::fromImage(qimage));
}

void StereoGUI::updatePoints(PointCloudT::Ptr points)
{
    if (!cloud_viewer->updatePointCloud(points, "cloud"))
    {
        cloud_viewer->addPointCloud(points, "cloud");
    }
    ui->qvtkCloud->update();
}

StereoGUI::~StereoGUI()
{
    this->close();
    delete ui;
}