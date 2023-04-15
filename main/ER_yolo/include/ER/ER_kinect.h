#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <yolo_opencv.h>

struct Intrinsic_Parameter
{
    double fx;
    double fy;
    double cx;
    double cy;
    double factor;
};  /*彩色相机内参(color_camera)*/
static Intrinsic_Parameter intrinsic_parameter[1] = {{608.329102, 608.140869, 640.645386, 363.350769, 1000.0}};

struct point
{
    float x;
    float y;
    float z;
    int row;
    int col;
    int index;
};





struct output
{
    cv::Point center;
    cv::Rect rect;
    float x = 0;
    float y = 0;
    int row = 0;
    int col = 0;
    float distance = 0;
    float height = 0.0;
    int index = 0;
    char tag = 'N';
    cv::Mat roi;
    float horizontal_angle = 0;
    float elevation_angle = 0;
    cv::Mat image;
    int cur_count = 0;
    point points[6000];
};
static output results[8];


pcl::PointCloud<pcl::PointXYZRGB>::Ptr   TRANS_TO_CLOUD (cv::Mat depth_image);

void CAMERA1_GET_INFORMATION(std::vector<YOLO::yolo_output> &results, cv::Mat depth_image_ocv , cv::Mat color_image_ocv);

/**
 * @brief 顶环颜色识别
 */
void COLOR_detect(cv::Mat color_image_ocv);

bool  CAMERA_DISTINGUISH_CYLINDER_ER(std::vector<YOLO::yolo_output> &results, int &b_diff_r, int &r_diff_g, int &temp_b, int &temp_r);

void Sort_Point();
