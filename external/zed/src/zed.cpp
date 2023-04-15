#include <zed/zed.h>

/**
 * @brief 构造函数
 */
zed::CAMERA::CAMERA(std::string zed_yaml)
{
    cv::FileStorage file(zed_yaml, cv::FileStorage::READ);

    if(file.isOpened())
    {
        file["brightness"] >> yaml_zed_settings.brightness;         // 亮度
        file["contrast"] >> yaml_zed_settings.contrast;             // 对比度
        file["hue"] >> yaml_zed_settings.hue;                       // 色调
        file["saturation"] >> yaml_zed_settings.saturation;         // 饱和度
        file["sharpness"] >> yaml_zed_settings.sharpness;           // 锐度
        file["gamma"] >> yaml_zed_settings.gamma;                   // 伽马矫正
        file["gain"] >> yaml_zed_settings.gain;                     // 曝光增益
        file["exposure"] >> yaml_zed_settings.exposure;             // 曝光时间
        file["white_balance"] >> yaml_zed_settings.white_balance;   // 白平衡
    }

    file.release();
}

/**
 *@brief 初始化zed相机
*/
void zed::CAMERA::INIT()
{
    /*设置参数*/
    sl::InitParameters init_params;
    /*设置相机分辨率为1080*/
    init_params.camera_resolution = yaml_zed_settings.camera_resolution;
    /*设置相机帧率为30*/
    init_params.camera_fps = yaml_zed_settings.camera_fps;

    /*设置要用于 SDK 的所有指标值的单位为m*/
    init_params.coordinate_units = sl::UNIT::METER;
    /*设置Camera用于返回其坐标系测量值的参考坐标系为右手系*/
    init_params.coordinate_system = yaml_zed_settings.coordinate_system;
    /*设置深度模式*/
    init_params.depth_mode = yaml_zed_settings.depth_mode;

    /*设置深度最小值*/
    init_params.depth_minimum_distance = yaml_zed_settings.minimum_depth;
    /*设置深度最大值*/
    init_params.depth_maximum_distance = yaml_zed_settings.maximum_depth;

    /*打开相机*/
    sl::ERROR_CODE returned_state = camera.open(init_params);
    if (returned_state != sl::ERROR_CODE::SUCCESS)
    {
        std::cout << "Error " << returned_state << ", exit program.\n";
        exit(1);
    }

    /*设置runtime参数*/
    runtime_params = sl::RuntimeParameters();
    //runtime_params.enable_fill_mode = false ;
    /*设置点云置信度*/
    runtime_params.confidence_threshold = 60;
    runtime_params.texture_confidence_threshold = 85;

    image_size = camera.getCameraInformation().camera_configuration.resolution;
    new_width = image_size.width;
    new_height = image_size.height;

    zed::CAMERA::SET_CAMERA();
}

/**
 *@brief 设置zed相机参数
*/
void zed::CAMERA::SET_CAMERA()
{
    camera.setCameraSettings(sl::VIDEO_SETTINGS::BRIGHTNESS, yaml_zed_settings.brightness);                   /*图片亮度*/
    camera.setCameraSettings(sl::VIDEO_SETTINGS::CONTRAST, yaml_zed_settings.contrast);                       /*对比度*/
    camera.setCameraSettings(sl::VIDEO_SETTINGS::HUE, yaml_zed_settings.hue);                                 /*色调*/
    camera.setCameraSettings(sl::VIDEO_SETTINGS::SATURATION, yaml_zed_settings.saturation);                   /*饱和度*/
    camera.setCameraSettings(sl::VIDEO_SETTINGS::SHARPNESS, yaml_zed_settings.sharpness);                     /*锐度*/
    camera.setCameraSettings(sl::VIDEO_SETTINGS::GAMMA, yaml_zed_settings.gamma);                             /*伽马矫正*/
    camera.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE, yaml_zed_settings.white_balance);  /*自动白平衡*/
    camera.setCameraSettings(sl::VIDEO_SETTINGS::GAIN, yaml_zed_settings.gain);                               /*曝光增益*/
    camera.setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, yaml_zed_settings.exposure);                       /*曝光时间*/
}

/**
 *@brief 获取zed相机图像
 * @return vector<cv::Mat> pictures 表示获取到的图像； pictures[0] 彩色图像； pictures[1] 深度图像；
 */
std::vector<cv::Mat> zed::CAMERA::GET_IMG(){
    sl::Mat depth_image_camera(new_width, new_height, sl::MAT_TYPE::U8_C4,sl::MEM::GPU);
    sl::Mat image_camera(new_width, new_height, sl::MAT_TYPE::U8_C4);

    if (camera.grab(runtime_params) == sl::ERROR_CODE::SUCCESS) /*判断是否捕获成功*/
    {
        camera.retrieveImage(depth_image_camera, sl::VIEW::DEPTH, sl::MEM::GPU, image_size);    /*捕获深度图*/
        camera.retrieveImage(image_camera, sl::VIEW::LEFT, sl::MEM::CPU, image_size);   /*捕获彩色图*/
    }

    cv::Mat image_ocv = SL_Mat2CV_MAT(image_camera);  /*将sl类型转换为opencv类型*/
    cv::cuda::GpuMat depth_image_ocv = SL_Mat2CV_MAT_GPU(depth_image_camera);  /*深度图转换采用gpu加速*/
    cv::Mat depth_image_ocv_m;

    depth_image_ocv.download(depth_image_ocv_m);
    std::vector<cv::Mat> pictures;
    pictures.push_back(image_ocv);
    pictures.push_back(depth_image_ocv_m);

    return pictures;
}

/**
 * @brief 获取opencv图像类型
 * @param type sl图像类型
 * @return opencv图像类型
 */
int zed::CAMERA::GET_OCV_TYPE(sl::MAT_TYPE type)
{
    int cv_type = -1;
    switch (type)
    {
        /*将sl数据类型转一一对应换为opencv*/
        case sl::MAT_TYPE::F32_C1:
            cv_type = CV_32FC1;
            break;
        case sl::MAT_TYPE::F32_C2:
            cv_type = CV_32FC2;
            break;
        case sl::MAT_TYPE::F32_C3:
            cv_type = CV_32FC3;
            break;
        case sl::MAT_TYPE::F32_C4:
            cv_type = CV_32FC4;
            break;
        case sl::MAT_TYPE::U8_C1:
            cv_type = CV_8UC1;
            break;
        case sl::MAT_TYPE::U8_C2:
            cv_type = CV_8UC2;
            break;
        case sl::MAT_TYPE::U8_C3:
            cv_type = CV_8UC3;
            break;
        case sl::MAT_TYPE::U8_C4:
            cv_type = CV_8UC4;
            break;
        default:
            break;
    }
    return cv_type;
}

/**
 * @brief 获取opencv图像
 * @param input sl图像
 * @return opencv图像
 */
cv::Mat zed::CAMERA::SL_Mat2CV_MAT(sl::Mat& input)
{
    cv::Mat res_image = cv::Mat(input.getHeight(), input.getWidth(), GET_OCV_TYPE(input.getDataType()), input.getPtr<sl::uchar1>(sl::MEM::CPU), input.getStepBytes(sl::MEM::CPU));
    //cv::cvtColor(res_image, res_image, COLOR_RGBA2RGB); // 从四通道转到三通道
    return res_image;
}

/**
 * @brief 获取opencv GpuMat图像
 * @param input sl图像
 * @return opencv GpuMat图像
 */
cv::cuda::GpuMat zed::CAMERA::SL_Mat2CV_MAT_GPU(sl::Mat& input)
{
    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::cuda::GpuMat(input.getHeight(), input.getWidth(), GET_OCV_TYPE(input.getDataType()), input.getPtr<sl::uchar1>(sl::MEM::GPU), input.getStepBytes(sl::MEM::GPU));
}

/**
 * @brief 获取神经网络识别到的结果的所需信息
 * @param results 神经网络获取到的结果
 * @param point_cloud 相机捕获到的点云
 */
void zed::CAMERA::CAMERA_GET_INFORMATION(std::vector<output> &results)
{
    for(int i = 0; i < results.size(); i ++)
    {
        /*选取框的中心点作为目标点*/
        int x = (results[i].kpts.kpt[2].x + results[i].kpts.kpt[3].x + results[i].kpts.kpt[4].x + results[i].kpts.kpt[5].x)/4;
        int y = (results[i].kpts.kpt[2].y + results[i].kpts.kpt[3].y + results[i].kpts.kpt[4].y + results[i].kpts.kpt[5].y)/4;
        sl::float4 point_cloud_value;
        /*获取目标点的坐标*/
        point_cloud.getValue(x, y, &point_cloud_value);
        if(!isnan(point_cloud_value.x) && !isnan(point_cloud_value.y) && !isnan(point_cloud_value.z) && point_cloud_value.x != 0 && point_cloud_value.y != 0 && point_cloud_value.z != 0)
        {
            results[i].height = (0.4 + point_cloud_value.y) * 2 ;
            results[i].distance = sqrt(powf(point_cloud_value.x, 2) + powf(point_cloud_value.z, 2));         /*得到目标物体的距相机左摄像头的距离*/
            results[i].x = point_cloud_value.x;              /*得到相机的x，y坐标，为区分柱子*/
            results[i].y = abs(point_cloud_value.z);
            results[i].horizontal_angle = acos(float(results[i].x / sqrt(powf(results[i].x,2) + powf((results[i].y-0.36), 2))));
        }
        else
        {
            results.erase(results.begin() + i);
            i--;
        }
    }
}

/**
* @brief 捕获图片
*/
bool zed::CAMERA::CAPTURE()
{
    /* grab an image */
    if (camera.grab() == sl::ERROR_CODE::SUCCESS)
    {
        camera.retrieveImage(imageMap, sl::VIEW::LEFT); // Get the image if necessary
        camera.retrieveMeasure(depthMap, sl::MEASURE::DEPTH, sl::MEM::CPU); // Get the depth map
        camera.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA);

        this->color_image = SL_Mat2CV_MAT(imageMap);
        cv::cvtColor(color_image, color_image, cv::COLOR_RGBA2RGB);
        depth_image = SL_Mat2CV_MAT(depthMap);
        return true;
    }
    return false;
}

/**
 * @brief 关闭相机
 */
void zed::CAMERA::CLOSE()
{
    camera.close();
}

/**
* @brief 析构函数
*/
zed::CAMERA::~CAMERA()
{
    CLOSE();
}

/**
 * @brief  This function convert a RGBA color packed into a packed RGBA PCL compatible format
 **/
inline float convertColor(float colorIn)
{
    uint32_t color_uint = *(uint32_t *) & colorIn;
    unsigned char *color_uchar = (unsigned char *) &color_uint;
    color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);
    return *reinterpret_cast<float *> (&color_uint);
}
