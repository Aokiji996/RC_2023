#include <kinect/kinect.h>


static k4a::CAMERA*
        camera_ptr = nullptr;

k4a::CAMERA::CAMERA(){}
/**
 * @brief 相机构造函数
 */
k4a::CAMERA::CAMERA(std::string kinect_yaml)
{
    cv::FileStorage file(kinect_yaml, cv::FileStorage::READ);

    if(file.isOpened())
    {
        file["exposure_time_absolute"] >> yaml_kinect_setting.exposure_time_absolute; //曝光时间
        std::cout << yaml_kinect_setting.exposure_time_absolute << std::endl;
        file["brightness"] >> yaml_kinect_setting.brightness;                         //亮度
        file["contrast"] >> yaml_kinect_setting.contrast;                             //对比度
        file["saturation"] >> yaml_kinect_setting.saturation;                         //饱和度
        file["sharpness"] >> yaml_kinect_setting.sharpness;                           //锐度
        file["white_balance"] >> yaml_kinect_setting.white_balance;                   //白平衡
        file["backlight_compensation"] >> yaml_kinect_setting.backlight_compensation; //背光补偿
        file["gain"] >> yaml_kinect_setting.gain;                                     //增益
        file["powerline_frequency"] >> yaml_kinect_setting.powerline_frequency;       //电力线频率
    }

    file.release();
}
/**
 *@brief 初始化KinectAzureDK相机
 */
void k4a::CAMERA::INIT()
{
    /*查询设备数量*/
    uint32_t devices_count = k4a::device::get_installed_count();
    if (devices_count == 0)
    {
        printf("No K4a Devices Attached!\n");
        return ;
    }
    else
    {
        printf("Found %u Kinect Devices!\n", devices_count);
    }

    /*设置参数*/
    k4a_device_configuration_t init_params = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;

    /*设置color图的分辨率位 1080P*/
    init_params.color_resolution = K4A_COLOR_RESOLUTION_720P;
    /*设置Kinect的相机帧率为30FPS*/
    init_params.camera_fps = K4A_FRAMES_PER_SECOND_30;
    /*设置Kinect的深度模式为Near FOV unbinned（这一代 Kinect 支持多种深度模式，官方文档推荐使用 K4A_DEPTH_MODE_NFOV_UNBINNED 和 K4A_DEPTH_MODE_WFOV_2X2BINNED 这两种深度模式）*/
    init_params.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    /*设置Kinect的颜色属性为BGRA32*/
    init_params.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    /*为了同时获取depth和color图，保持这两种图像是同步的*/
    init_params.synchronized_images_only = true;

    /*打开相机*/
    int device_id_ = 0;
    camera = k4a::device::open(device_id_);
    camera.start_cameras(&init_params);

    /*查询设备SN码*/
    std::string serial_number = camera.get_serialnum();
    std::cout << "Open Kinect Device Serial Number: " << serial_number << std::endl;

    /*是否成功打开相机*/
    if(!camera)
    {
        printf("Kinect Open Error!\n");
        return ;
    }
    /*获取相机参数*/
    calibration = camera.get_calibration(init_params.depth_mode, init_params.color_resolution);
    transformation = k4a::transformation(calibration);

    /*初始化camera_ptr*/
    if(camera_ptr == nullptr)
    {
        camera_ptr = this;
    }
}


/**
 *@brief 设置KinectAzureDK相机参数
 */
void k4a::CAMERA::SET_CAMERA()
{
    camera.set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_AUTO, yaml_kinect_setting.exposure_time_absolute);    //曝光时间
//    camera.set_color_control(K4A_COLOR_CONTROL_BRIGHTNESS, K4A_COLOR_CONTROL_MODE_MANUAL, yaml_kinect_setting.brightness);                          // 亮度
//    camera.set_color_control(K4A_COLOR_CONTROL_CONTRAST, K4A_COLOR_CONTROL_MODE_MANUAL, yaml_kinect_setting.contrast);                              // 对比度
//    camera.set_color_control(K4A_COLOR_CONTROL_SATURATION, K4A_COLOR_CONTROL_MODE_MANUAL, yaml_kinect_setting.saturation);                          // 饱和度
//    camera.set_color_control(K4A_COLOR_CONTROL_SHARPNESS, K4A_COLOR_CONTROL_MODE_MANUAL, yaml_kinect_setting.sharpness);                            // 锐度                       // 锐度
//    camera.set_color_control(K4A_COLOR_CONTROL_WHITEBALANCE, K4A_COLOR_CONTROL_MODE_MANUAL, yaml_kinect_setting.white_balance);                     // 白平衡
//    camera.set_color_control(K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION, K4A_COLOR_CONTROL_MODE_MANUAL, yaml_kinect_setting.backlight_compensation);  // 背光补偿
//    camera.set_color_control(K4A_COLOR_CONTROL_GAIN, K4A_COLOR_CONTROL_MODE_MANUAL, yaml_kinect_setting.gain);                                      // GAIN
//    camera.set_color_control(K4A_COLOR_CONTROL_POWERLINE_FREQUENCY, K4A_COLOR_CONTROL_MODE_MANUAL, yaml_kinect_setting.powerline_frequency);        // 电力线频率
}

/**
 *@brief KinectAzureDK相机原始数据捕获
 * @return 是否成功捕获图像
 */
bool k4a::CAMERA::KINECT_AZURE_DK_SOURCE_GRABBER(uint8_t timeout_ms)
{
    timeval capture_start_time;
    gettimeofday(&capture_start_time, NULL);
    /*超时检测*/
    if(!camera.get_capture(&capture, std::chrono::milliseconds(100)))
    {
        printf("KinectAzureDK Grabber Failed!\n");
        return false;
    }

    color_image = capture.get_color_image();
    depth_image = capture.get_depth_image();
    if(color_image == nullptr || depth_image == nullptr)
        return false;

    timeval capture_now_time;
    gettimeofday(&capture_now_time, NULL);
    if(CAL_DIFF_TIME_MS(capture_start_time, capture_now_time) > timeout_ms)
    {
        printf("Grabber Time Out!\n");
        return false;
    }
    return true;
}

/**
 * @brief 获取Kinect相机图像
 * @param timeout_ms 超时时间
 * @param depthImage_k4a 深度图
 * @return std::vector<cv::Mat> pictures 表示获取到的图像； pictures[0] 彩色图像； pictures[1] 深度图像； pictures[2] 红外线图像
 */
std::vector<cv::Mat> k4a::CAMERA::GET_IMG(uint8_t timeout_ms)
{
    /*清空上一帧数据*/
    for(int i = 0; i < 8; i++)
    {
        output cover;
        results[i] = cover;
    }
    /*获取相机原始数据*/
    cv::Mat colorImage_ocv, depthImage_ocv;
    depth_image = nullptr, color_image = nullptr;
    bool isCapture = false;
    isCapture = KINECT_AZURE_DK_SOURCE_GRABBER(timeout_ms);
    /*深度图和彩色图配准*/
    std::vector<cv::Mat> pictures;

    /*检验是否捕获成功*/
    if(!isCapture)
        return pictures;
    /*数据格式转换*/
    if(color_image != nullptr)
    {
        color_image_ocv = cv::Mat(color_image.get_height_pixels(), color_image.get_width_pixels(), CV_8UC4, (void *)color_image.get_buffer());
        cvtColor(color_image_ocv, color_image_ocv, cv::COLOR_BGRA2BGR); // 从四通道转到三通道
    }
    if(depth_image != nullptr)
    {
        k4a::image transformed_depth_image = transformation.depth_image_to_color_camera(depth_image);
        depthImage_ocv = cv::Mat(transformed_depth_image.get_height_pixels(), transformed_depth_image.get_width_pixels(), CV_16UC1, (void *)transformed_depth_image.get_buffer());
    }
    pictures.push_back(color_image_ocv);
    pictures.push_back(depthImage_ocv);

    depth_image_ocv=depthImage_ocv.clone();

    depthImage_ocv.convertTo(depthImage_ocv, CV_8U, 1);
    return pictures;
}

/**
 * @brief 获取中间六根柱子的点云图并且分类
 * @param depth_image 深度图
 * @return 中间六根柱子的点云图
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr k4a::CAMERA::TRANS_TO_CLOUD(cv::Mat depth_image)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr return_colum_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if(depth_image.empty())
        return return_colum_point_cloud;

    for(int x = 0; x < depth_image.cols; x++)
    {
        for(int y = 0; y < depth_image.rows; y++)
        {
            pcl::PointXYZRGB p;
            ushort d = depth_image.at<ushort>(y, x);
            float z1 = (float)d;
            z1 /= 1000.f;

            if(z1 == 0 || isnan(z1) || z1 > 7.0)
                continue;

            float x1 = (x - intrinsic_parameter[0].cx) * z1 / intrinsic_parameter[0].fx;
            float y1 = (y - intrinsic_parameter[0].cy) * z1 / intrinsic_parameter[0].fy;
            y1 = -y1;
            if(y1 < -0.3)
                continue;
            p.x = x1;
            p.y = z1;
            p.z = y1;
            /*向右为x轴正方向，向前为y轴正方向，向上为z轴正方向*/
            p.b = color_image_ocv.at<cv::Vec3b>(y, x)[0];
            p.g = color_image_ocv.at<cv::Vec3b>(y, x)[1];
            p.r = color_image_ocv.at<cv::Vec3b>(y, x)[2];

            if(y1 > 0.6 && y1 < 1.0 && z1 > 3.0 && z1 < 6.0) /*分类中间1号柱子点*/
            {
                results[0].points[results[0].cur_count] = {x1, z1, y1, y, x, 1};
                results[0].x += x1;
                results[0].y += z1;
                results[0].row += y;
                results[0].col += x;
                results[0].cur_count = (results[0].cur_count+1)%6000;
                return_colum_point_cloud->points.push_back(p);
            }
            else if(y1 > -0.1 && y1 < 0.3 && x1 < -0.5 && z1 < 5.0) /*分类2号柱子点*/
            {
                results[1].points[results[1].cur_count] = {x1, z1, y1, y, x, 2};
                results[1].x += x1;
                results[1].y += z1;
                results[1].row += y;
                results[1].col += x;
                results[1].cur_count = (results[1].cur_count+1)%6000;
                return_colum_point_cloud->points.push_back(p);
            }
            else if(y1 > -0.1 && y1 < 0.3 && x1 > 0.5 && z1 < 5.0) /*分类3号柱子点*/
            {
                results[2].points[results[2].cur_count] = {x1, z1, y1, y, x, 3};
                results[2].x += x1;
                results[2].y += z1;
                results[2].row += y;
                results[2].col += x;
                results[2].cur_count = (results[2].cur_count+1)%6000;
                return_colum_point_cloud->points.push_back(p);
            }
            else if(y1 > -0.1 && y1 < 0.3 && x1 < -0.5 && z1 > 4.0) /*分类4号柱子点*/
            {
                results[3].points[results[3].cur_count] = {x1, z1, y1, y, x, 4};
                results[3].x += x1;
                results[3].y += z1;
                results[3].row += y;
                results[3].col += x;
                results[3].cur_count = (results[3].cur_count+1)%6000;
                return_colum_point_cloud->points.push_back(p);
            }
            else if(y1 > -0.1 && y1 < 0.3 && x1 > 0.5 && z1 > 4.0) /*分类5号柱子点*/
            {
                results[4].points[results[4].cur_count] = {x1, z1, y1, y, x, 5};
                results[4].x += x1;
                results[4].y += z1;
                results[4].row += y;
                results[4].col += x;
                results[4].cur_count = (results[4].cur_count+1)%6000;
                return_colum_point_cloud->points.push_back(p);
            }
            else if(z1 > 1.0 && z1 < 2.0 && y1 > -0.3 && y1 < 0.1) /*分类6号柱子点*/
            {
                results[5].points[results[5].cur_count] = {x1, z1, y1, y, x, 6};
                results[5].x += x1;
                results[5].y += z1;
                results[5].row += y;
                results[5].col += x;
                results[5].cur_count = (results[5].cur_count+1)%6000;
                return_colum_point_cloud->points.push_back(p);
            }
        }
    }

    if (return_colum_point_cloud->size() == 0)
        return return_colum_point_cloud;

    /*无序点云*/
    return_colum_point_cloud->width = return_colum_point_cloud->size();
    return_colum_point_cloud->height = 1;
    return_colum_point_cloud->is_dense = false;
    return return_colum_point_cloud;
}

/**
   * @brief 获取神经网络识别到的结果的所需信息
   * @param results 神经网络获取到的结果
   * @param point_cloud 相机捕获到的点云
   */
void k4a::CAMERA::CAMERA_GET_INFORMATION()
{
    /*求1-6号柱子的信息*/
    for(int i = 0; i < 6; i++)
    {
        if(results[i].cur_count == 0)
        {
            continue;
        }
        results[i].x = results[i].x / results[i].cur_count;
        results[i].y = results[i].y / results[i].cur_count;
        results[i].col = results[i].col / results[i].cur_count;
        results[i].row = results[i].row / results[i].cur_count;
        results[i].distance = sqrt(pow(results[i].x, 2) + pow(results[i].y, 2));
        results[i].index = i + 1;
    }
}


/**
 * @brief 顶环颜色识别
 */
void k4a::CAMERA::COLOR_DETECT()
{
    for(int n = 0; n < 6; n++)
    {
        point change[6000];
        int count = 0;
        for(int k = 0; k < results[n].cur_count; k ++)
        {
            if(fabs(results[n].points[k].x - results[n].x) < 0.02)
            {
                change[count++] = results[n].points[k];
            }
        }
        results[n].cur_count = count;
        std::copy(std::begin(change), std::end(change), std::begin(results[n].points));
    }

    for(int n = 0; n < 6; n++)
    {

        int min_red_rows[900]={0};
        int min_blue_rows[900]={0};
        int min_cout_rows[900]={0};

        for(int i = 0; i < results[n].cur_count; i++)
        {
            int row = results[n].points[i].row;
            int col = results[n].points[i].col;

            int b = color_image_ocv.at<cv::Vec3b>(row, col)[0];
            int g = color_image_ocv.at<cv::Vec3b>(row, col)[1];
            int r = color_image_ocv.at<cv::Vec3b>(row, col)[2];

            if (-675*r-286*g+665*b>5542)
            {
                color_image_ocv.at<cv::Vec3b>(row, col)[0] = 255;
                color_image_ocv.at<cv::Vec3b>(row, col)[1] = 0;
                color_image_ocv.at<cv::Vec3b>(row, col)[2] = 0;

                min_blue_rows[row]+=255;
                min_red_rows[row]+=0;
                min_cout_rows[row]= (min_cout_rows[row]+1) % 900;
            }
            else if ( ((4577*r -2339 * g +964 *b > 620000)&& ( -2186.17 * r +6087 *g - 5755.68 *b < -178973.39)&&(-109.92 * r + 1536.96 *g +1511.56 * b <500000))
                        ||((2485.5 * r - 5237.5 * g + 363 *b >0) &&(22*r-27*g+11*b  > 1305)))
            {

                color_image_ocv.at<cv::Vec3b>(row, col)[0] = 0;
                color_image_ocv.at<cv::Vec3b>(row, col)[1] = 0;
                color_image_ocv.at<cv::Vec3b>(row, col)[2] = 255;
                min_blue_rows[row]+=0;
                min_red_rows[row]+=255;
                min_cout_rows[row]= (min_cout_rows[row]+1) % 900;
            }
            else
            {
                color_image_ocv.at<cv::Vec3b>(row, col)[0] = 0;
                color_image_ocv.at<cv::Vec3b>(row, col)[1] = 0;
                color_image_ocv.at<cv::Vec3b>(row, col)[2] = 0;
            }
        }

        for(int i=0;i<this->color_image_ocv.rows;i++)
        {
            if(min_cout_rows[i]==0)
                continue;
            float temp = (min_red_rows[i] - min_blue_rows[i]) / min_cout_rows[i];
            if(temp > 40)
            {
                results[n].tag = 'R';
                break;
            }
            else if(temp < -100)
            {
                results[n].tag = 'B';
                break;
            }
        }
        std::string label = std::to_string(results[n].index) + results[n].tag;
        int base_line;
        cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &base_line);
        int label_top = cv::max(results[n].center.y - 6 , label_size.height);
        int label_left = results[n].center.x + 10;
        cv::putText(this->color_image_ocv, label, cv::Point(label_left - 1, label_top - 2),cv::FONT_HERSHEY_SIMPLEX, 1,(0, 255, 0), 3);
    }
}

/**
 * @brief 计算时间差
 * @param first_time
 * @param second_time
 * @return 时间差
 */
double k4a::CAMERA::CAL_DIFF_TIME_MS(timeval first_time, timeval second_time)
{
    double diff_time_ms = 1e3 * (second_time.tv_sec - first_time.tv_sec) + (second_time.tv_usec - first_time.tv_usec) / 1000.0;
    if(diff_time_ms < 0)
    {
        diff_time_ms *= -1.0;
    }
    return diff_time_ms;
}


/**
 * @brief 关闭相机
 */
void k4a::CAMERA::CLOSE()
{
    camera.close();
}

/**
 * @brief 相机析构函数
 */
k4a::CAMERA::~CAMERA()
{
    CLOSE();
}
