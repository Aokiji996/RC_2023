#include <ER/main.h>

com::FRAME f;
com::USART u1;

bool ctrl_c_pressed = false;
void ctrlc(int)
{
    ctrl_c_pressed = true;
    printf("Ctrl C!\n");
}

int main(int argc, char **argv)
{

    for(int i = 0; i < 8; i++)
    {
        f.FRAME_ADD_ELEMENT(com::is_char);
        f.FRAME_ADD_ELEMENT(com::is_int);
        f.FRAME_ADD_ELEMENT(com::is_float);
        f.FRAME_ADD_ELEMENT(com::is_float);
        f.FRAME_ADD_ELEMENT(com::is_char);
    }
    f.FRAME_ADD_ELEMENT(com::is_char);
    f.FRAME_PRINT_ELEMENTS();

    /* 接收数据设置 */
    //  std::function<void(char *, int, com::FRAME)> foo = get_receive;
    // u1.USART_SET_ReceiveCallback_Frame(foo);

    std::string com_yaml = "/home/rc/下载/RC_2023/config/com.yaml";
    YAML::Node com_config = YAML::LoadFile(com_yaml);
    cout << " uart_uart_name " << com_config["uart_uart_name"].as<string>() << endl;
    string uart_name = com_config["uart_uart_name"].as<string>();

    u1.USART_SET_COM_NAME(uart_name);
    u1.USART_SET_RECEIVER_MODE(true);
    u1.USART_GET_FRAME(f);
    u1.USART_INIT();



    std::string lader_yaml = "/home/rc/下载/RC_2023/config/slamtic.yaml";

    s2::LIDAR lader(lader_yaml);
    lader.LIDAR_INIT();
    lader.LIDAR_START_THREAD_CAPTURE();

    std::string kinect_yaml = "/home/rc/下载/RC_2023/config/kinect.yaml";
    k4a::CAMERA camera2(kinect_yaml);
    camera2.INIT();
    camera2.SET_CAMERA();

    signal(SIGINT, ctrlc);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("Viewer_PCL1"));
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("Viewer_PCL2"));
    //设置默认的坐标系/
    //设置固定的元素。红色是X轴，绿色是Y轴，蓝色是Z轴。

    viewer1->addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(10, 0, 0), "x");
    viewer1->addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 5, 0), "y");
    viewer1->addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 0, 2), "z");

    viewer2->addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(10, 0, 0), "x");
    viewer2->addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 5, 0), "y");
    viewer2->addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 0, 2), "z");

    while(1){

        if(ctrl_c_pressed)
            break;

        timeval capture_start_time;
        gettimeofday(&capture_start_time, NULL);

        std::vector<cv::Mat> pictures;
        pictures = camera2.GET_IMG(100);
        if(!pictures.size())
            continue;
        pcl::PointCloud<pcl::PointXYZ>::Ptr vis_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);

        vis_cloud2=TRANS_TO_CLOUD(camera2.depth_image_ocv);

        CAMERA1_GET_INFORMATION();

        COLOR_detect(camera2.color_image_ocv);

        cv::imshow("color", camera2.color_image_ocv);
        cv::waitKey(1);

        //vis_cloud1 = LIDAR_LOCATION_TARGETING(results , &lader);
        CAMERA_SEND_INFORMATION_ER(results);

        o7.x=0.01;
        o7.y=0.01;

        o8.x=0.01;
        o8.y=0.01;

        timeval capture_now_time;
        gettimeofday(&capture_now_time, NULL);
        double diff_time_ms = 1e3*(capture_now_time.tv_sec - capture_start_time.tv_sec) + (capture_now_time.tv_usec - capture_start_time.tv_usec)/1000.0;

        std::cout << diff_time_ms << std::endl;


    }

    camera2.CLOSE();
    return 0;
}
