#include <ER/main.h>

com::FRAME f;
com::USART u1;

bool if_change = true;
char change[3];
float a[3];
using namespace cv;
using namespace std;

void get_receive(char *data, int length, com::FRAME frame); //串口接收回调

pthread_t id_thread1;
//void* thread1(void *arg);

// 互斥锁
pthread_mutex_t mutex_count = PTHREAD_MUTEX_INITIALIZER;



// std::string kinect_yaml = "/home/rc/下载/RC_2023/config/kinect.yaml";
//k4a::CAMERA camera2(kinect_yaml);
cv::Mat colorImage_ocv, depthImage_ocv;

YOLO::yolo_output resultes_for_save[9];
YOLO::yolo_output resultes_used_for_deteceted[9];

int start_capture = 0;
bool detect_if_need_yolo= false;



bool ctrl_c_pressed = false;
void ctrlc(int)
{
    ctrl_c_pressed = true;
    printf("Ctrl C!\n");
}

int main(int argc, char **argv)
{
//    pthread_create(&id_thread1, NULL, thread1, NULL);

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
    std::function<void(char *, int, com::FRAME)> foo = get_receive;
    u1.USART_SET_ReceiveCallback_Frame(foo);

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

    std::string yolo_yml="/home/rc/下载/RC_2023/config/yolov7-k6.yaml";
    YOLO test(yolo_yml);

    test.YOLO_INIT();


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


    float statc_time = 0.0;
    float detec_if_same = 0.0;

    int b_diff_r = 35;
    int r_diff_g = 80;
    int temp_r = 40;
    int temp_b = -120;
    cv::namedWindow("detect");
    cv::createTrackbar("b_diff_r", "detect", &b_diff_r, 100);
    cv::createTrackbar("r_diff_g", "detect", &r_diff_g, 120);
    cv::createTrackbar("temp_r", "detect", &temp_r, 100);
    cv::createTrackbar("temp_b", "detect", &temp_b, 10);
    while(1){
        if(ctrl_c_pressed)
        {
            printf("Ctrl C!\n");
            break;
        }

        if(cv::waitKey(10) == 'q')
        {
            printf("q 退出!\n");
            break;
        }

        int cout_times = 0;

        if (if_change)
        {  //判断是否启动
            cout_times++;

            if(cout_times == 5)
            {
                detect_if_need_yolo = true;
                cout_times = 0;
            }

            timeval capture_start_time;
            gettimeofday(&capture_start_time, NULL);


            pcl::PointCloud<pcl::PointXYZ>::Ptr vis_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
            std::vector<cv::Mat> pictures = camera2.GET_IMG(100);

            if (camera2.color_image_ocv.empty() || camera2.depth_image_ocv.empty())
            {
                continue;
            }
            cv::imshow("color", camera2.color_image_ocv);

            if (detect_if_need_yolo)  //判断是否需要yolo 再次全画
            {
                test.results.clear();
                test.YOLO_KEY_POINTS_DETECT(camera2.color_image_ocv);
                detect_if_need_yolo = false;
            }
            else
            {
                test.results.clear();
                for (int j = 1; j < 7; j++)
                {
                    YOLO::yolo_output result1;
                    result1.id = resultes_for_save[j].id;
                    result1.confidence = resultes_for_save[j].confidence;
                    result1.bbox = resultes_for_save[j].bbox;
                    result1.kpts = resultes_for_save[j].kpts;
                    test.results.push_back(result1);
                }

            }
            CAMERA1_GET_INFORMATION(test.results, camera2.depth_image_ocv, camera2.color_image_ocv);
            if (test.results.size() > 5)
            {
                bool if_process_right = true;
                if_process_right = CAMERA_DISTINGUISH_CYLINDER_ER(test.results, b_diff_r, r_diff_g, temp_b, temp_r);

                if (!if_process_right)
                {
                    continue;
                }
            }
            else
            {
                detect_if_need_yolo = true;
                continue;
            }

            for (int k = 0; k < test.results.size(); k++)
            {

                resultes_used_for_deteceted[test.results[k].index].kpts = test.results[k].kpts;
                resultes_used_for_deteceted[test.results[k].index].bbox = test.results[k].bbox;
                resultes_used_for_deteceted[test.results[k].index].confidence = test.results[k].confidence;
                resultes_used_for_deteceted[test.results[k].index].id = test.results[k].id;
                resultes_used_for_deteceted[test.results[k].index].x = test.results[k].x;
                resultes_used_for_deteceted[test.results[k].index].y = test.results[k].y;

            }
            for (int m = 1; m < 7; m++)
            {
                if ((fabs(resultes_for_save[m].x - resultes_used_for_deteceted[m].x) > 0.1) ||
                    (fabs(resultes_for_save[m].y - resultes_used_for_deteceted[m].y) > 0.1))
                    detect_if_need_yolo = true;
            }

            //将本帧数据保留给下一镇
            for (int n = 0; n < 7; n++)
            {
                resultes_for_save[n].kpts = resultes_used_for_deteceted[n].kpts;
                resultes_for_save[n].id = resultes_used_for_deteceted[n].id;
                resultes_for_save[n].confidence = resultes_used_for_deteceted[n].confidence;
                resultes_for_save[n].bbox = resultes_used_for_deteceted[n].bbox;
                resultes_for_save[n].x = resultes_used_for_deteceted[n].x;
                resultes_for_save[n].y = resultes_used_for_deteceted[n].y;
            }

            if (detect_if_need_yolo)
             continue;

            test.YOLO_OBJECT_DRAW_PRED(camera2.color_image_ocv);
            cv::imshow("detect",camera2.color_image_ocv);

            if (test.results.size() >= 6)
            {
                vis_cloud1=LIDAR_LOCATION_TARGETING(test.results , &lader);
            }
            else
            {
                continue;
            }

            CAMERA_SEND_INFORMATION_ER(test.results);

            viewer1->addSphere(o7, 0.5, "sphere1", 0); //添加圆球几何对象
            viewer1->addSphere(o8, 0.4, "sphere2", 0);

            viewer1->addPointCloud(vis_cloud1, "cloud2");
            viewer1->spinOnce(10);
            viewer1->removePointCloud("cloud2");

            viewer1->removeShape("sphere1");
            viewer1->removeShape("sphere2");

            o7.x = 0.01;
            o7.y = 0.01;

            o8.x = 0.01;
            o8.y = 0.01;

            timeval capture_now_time;
            gettimeofday(&capture_now_time, NULL);
            double diff_time_ms = 1e3 * (capture_now_time.tv_sec - capture_start_time.tv_sec) +
                                  (capture_now_time.tv_usec - capture_start_time.tv_usec) / 1000.0;
            statc_time += diff_time_ms;
            cout << statc_time << endl;
            statc_time = 0;

        }
        if(cv::waitKey(1)=='q')
        {
            exit(0);
        }
    }

    camera2.CLOSE();
    return 0;
}


//
//
//void* thread1(void *arg)
//{
//
//    camera2.INIT();
//    camera2.SET_CAMERA();
//
//    float statc_time=0.0;
//
//    while(1)
//    {
//        timeval capture_start_time;
//        gettimeofday(&capture_start_time, NULL);
//
//        std::vector<cv::Mat> pictures;
//        pictures = camera2.GET_IMG(100);
//
//
//        if(camera2.color_image_ocv.empty()){
//            timeval capture_now_time;
//            gettimeofday(&capture_now_time, NULL);
//            double diff_time_ms = 1e3*(capture_now_time.tv_sec - capture_start_time.tv_sec) + (capture_now_time.tv_usec - capture_start_time.tv_usec)/1000.0;
////        std::cout << diff_time_ms << std::endl;
//
//            statc_time += diff_time_ms;
//            continue;
//        }
//
//
//        pthread_mutex_lock(&mutex_count);
//        colorImage_ocv = camera2.color_image_ocv;
//        depthImage_ocv = camera2.depth_image_ocv;
//        pthread_mutex_unlock(&mutex_count);
//
//        //    cv::imwrite("/home/rc/下载/save/9.jpg",colorImage_ocv);
//
//        cv::imshow(" color ", colorImage_ocv);
//        cv::waitKey(10);
//
//        timeval capture_now_time;
//        gettimeofday(&capture_now_time, NULL);
//        double diff_time_ms = 1e3*(capture_now_time.tv_sec - capture_start_time.tv_sec) + (capture_now_time.tv_usec - capture_start_time.tv_usec)/1000.0;
////        std::cout << diff_time_ms << std::endl;
//        statc_time += diff_time_ms;
////        cout<< statc_time<<endl;
//        statc_time =0;
//
//    }
//}