#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>
#include <sl/Camera.hpp>

struct key_points
{
    cv::Point kpt[6];
};

struct output
{
    int class_id;
    float confidence;
    cv::Rect bbox;
    key_points kpts;

    float x;
    float y;
    float distance;
    float height=0.0001;
    int index = 0;

    float horizontal_angle;
    float elevation_angle;
};
namespace zed
{
    class CAMERA
    {
public:
   CAMERA(std::string zed_yaml);  // 构造函数
   ~CAMERA();  // 析构函数

/***********************设置相机***********************/
    private:
        static const size_t size_devList = 2;   /*相机数量*/

        sl::Resolution image_size;

        sl::RuntimeParameters runtime_params;   /*相机参数*/
        struct Yaml_Zed_Settings
        {
            int brightness;                          //亮度
            int contrast;                            //对比度
            int hue;                                 //色调
            int saturation;                          //饱和度
            int sharpness;                           //锐度
            int gamma;                               //伽马矫正
            int gain;                                //曝光增益
            int exposure;                            //曝光时间
            int white_balance;                       //白平衡
            sl::RESOLUTION camera_resolution;        //分辨率
            int camera_fps;                          //相机帧率
            sl::DEPTH_MODE depth_mode;               //相机深度模式
            float minimum_depth;                     //最小深度值
            float maximum_depth;                     //最大深度值
            sl::COORDINATE_SYSTEM coordinate_system; //坐标系
        }yaml_zed_settings;
    public:
        sl::Camera camera;
        sl::Mat imageMap, depthMap, point_cloud;
        cv::Mat color_image, depth_image;

        /**
         * @brief 初始化zed相机
        */
        void INIT();

        /**
         * @brief 设置zed相机参数
        */
        void SET_CAMERA();

        /**
         * @brief 关zed相机
         */
        void CLOSE();

        /**
         * @brief 获取opencv图像类型
         * @param type sl图像类型
         * @return opencv图像类型
         */
        int GET_OCV_TYPE(sl::MAT_TYPE type);

        /**
         * @brief 获取opencv图像
         * @param input sl图像
         * @return opencv图像
         */
        cv::Mat SL_Mat2CV_MAT(sl::Mat &input);

        /**
         * @brief 获取zed相机图像
         * @return vector<cv::Mat> pictures 表示获取到的图像； pictures[0] 彩色图像； pictures[1] 深度图像；
        */
        std::vector<cv::Mat> GET_IMG();

        /**
         * @brief 获取神经网络识别到的结果的所需信息
         * @param result 神经网络获取到的结果
         * @param point_cloud 相机捕获到的点云
         */
        void CAMERA_GET_INFORMATION(std::vector<output> &results);

        /**
         * 获取opencv GpuMat图像
         * @param input sl图像
         * @return opencv GpuMat图像
         */
        cv::cuda::GpuMat SL_Mat2CV_MAT_GPU(sl::Mat &input);

        /**
         * @brief 捕获图片
         * @return
         */
        bool CAPTURE();
    private:

        int new_width;
        int new_height;

    public:
        cv::cuda::GpuMat DepthMask(float dist, float range, cv::cuda::GpuMat temp)
        {
            sl::Mat point_cloud2;
            sl::float1 point;
            camera.retrieveMeasure(point_cloud2, sl::MEASURE::DEPTH, sl::MEM::GPU, image_size);
            cv::cuda::GpuMat depthMask;
            depthMask.upload(cv::Mat::zeros(temp.size(), CV_8UC1));
            for (int i = 0; i < temp.rows; i++)
            {
                for (int j = 0; j < temp.cols; j++)
                {
                    point_cloud2.getValue(0.5 * new_width - 150 + i, 0.5 * new_height - 150 + j, &point);
                    float distance;
                    distance = point;
                    if (abs(distance / 100 - dist) < range)
                    {
                        depthMask.ptr<uchar>(j)[i] = 1;
                    }
                }
            }
            return depthMask;
        }

        int GetWidth()
        {
            return new_width;
        }

        int GetHeight()
        {
            return new_height;
        }
    };
}
