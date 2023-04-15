#pragma once

#include <slamtec/slamtec.h>
#include <ER/ER_kinect.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>



struct point_pose{
    float distance=0;
    float angle=0;
    float ordinary_angle=0;
    float x=0;
    float y=0;
    int index=0;
    int point_nums=0;
    int select_colum_times=0;
    int colum_information_id=0;
};



static std::queue<point_pose> points;
static std::vector<point_pose> sort_points[200];
static point_pose colum_information[100];
static point_pose colums[2];

static pcl::PointXYZ o1;
static pcl::PointXYZ o2;
static pcl::PointXYZ o3;
static pcl::PointXYZ o4;
static pcl::PointXYZ o5;
static pcl::PointXYZ o6;
static pcl::PointXYZ o7;
static pcl::PointXYZ o8;

pcl::PointCloud<pcl::PointXYZ>::Ptr LIDAR_LOCATION_TARGETING(output *results , s2::LIDAR *lidar);

void Select_Colum_Information();

void select_colum();

void ER_compute_the_real_colum();