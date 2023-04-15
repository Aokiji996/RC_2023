#include <iostream>
#include <k4a/k4a.h>
#include <k4a/k4a.hpp>
#include <k4a/k4atypes.h>
#include <k4arecord/record.h>
#include <string>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <k4a/k4a.h>
#include <k4a/k4a.hpp>
#include <k4a/k4atypes.h>
#include <k4arecord/record.h>


#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pthread.h>

#include <sys/time.h>
#include <unistd.h>
#include <iostream>

namespace k4a {
    class CAMERA {
 public:
     CAMERA(std::string kinect_yaml); // 构造函数
     CAMERA();
     ~CAMERA();// 析构函数

        /***********************设置相机***********************/
    private:
        k4a::device camera;                 //相机
        k4a::calibration calibration;       //相机参数
        k4a::transformation transformation; //转换变量
        k4a::capture capture;               //捕获

        k4a::image depth_image;        /*捕获到的深度图*/
        k4a::image color_image;         /*捕获到的彩色图*/
    public:
        cv::Mat color_image_ocv, depth_image_ocv;
        struct point
        {
            float x;
            float y;
            float z;
            int row;
            int col;
            int index;
        }; //点云获取点的类型

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
        }; //识别结果类型
        output results[8]; //8个柱子的信息

        struct Intrinsic_Parameter
        {
            double fx;
            double fy;
            double cx;
            double cy;
            double factor;
        };  /*彩色相机内参(color_camera)*/
        Intrinsic_Parameter intrinsic_parameter[1] = {{608.329102, 608.140869, 640.645386, 363.350769, 1000.0}};

        struct Setting
        {
            int exposure_time_absolute;             //曝光时间
            int brightness;                         //亮度
            int contrast;                           //对比度
            int saturation;                         //饱和度
            int sharpness;                          //锐度
            int white_balance;                      //白平衡
            int backlight_compensation;             //背光补偿
            int gain;                               //曝光增益
            int powerline_frequency;                //电力线频率
        }yaml_kinect_setting; //相机设置

    public:
        /**
        *@brief 初始化zed相机
        */
        void INIT(); // 初始化KinectAzureDK相机

        /**
        *@brief 设置KinectAzureDK相机参数
        */
        void SET_CAMERA(); // 设置KinectAzureDK相机参数

        /**
         * @brief 获取识别信息
         * @param results
         */
        void CAMERA_GET_INFORMATION();

        /**
        *@brief KinectAzureDK相机原始数据捕获
        */
        bool KINECT_AZURE_DK_SOURCE_GRABBER(uint8_t timeout_ms);

        /**
        * @brief 获取Kinect相机图像
        * @param timeout_ms 超时时间
        * @param depthImage_k4a 深度图
        * @return std::vector<cv::Mat> pictures 表示获取到的图像； pictures[0] 彩色图像； pictures[1] 深度图像； pictures[2] 红外线图像
        */
        std::vector<cv::Mat> GET_IMG(uint8_t timeout_ms);

        /**
         * @brief 计算时间差
         * @param first_time
         * @param second_time
         * @return 时间差
         */
        double CAL_DIFF_TIME_MS(timeval first_time, timeval second_time);

        /**
         * @brief 获取中间六根柱子的点云图并且分类
         * @param depth_image 深度图
         * @return 中间六根柱子的点云图
         */
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr TRANS_TO_CLOUD(cv::Mat depth_image);

        /**
         * @brief 顶环颜色识别
         */
        void COLOR_DETECT();

        /**
         * @brief 关闭相机
         */
        void CLOSE();
    };
}
