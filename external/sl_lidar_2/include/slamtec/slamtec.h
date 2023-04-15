#include <sl_lidar.h>
#include "sl_lidar_driver.h"

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <unistd.h>

using namespace  std;
namespace s2{
    class LIDAR {
    private:
        std::string lidar_uart_name = "/dev/USBslam";
        sl::ILidarDriver *lidar;

        int lidar_mode = 1;
        int baud_rate = 1000000;

        bool if_lidar_thread_capture_work = false;


        pthread_t id_sl_lidar_capture;




    public:
        pthread_mutex_t mutex_sl_lidar_t = PTHREAD_MUTEX_INITIALIZER; //雷达收集点的互斥锁
        sl_lidar_response_measurement_node_hq_t lidar_nodes[8192]; //雷达所扫描到的点
        size_t node_count = 8192; //雷达所扫描到的点的数量

        LIDAR();

        /**
         * @brief 通过读取雷达 yaml 文件 启动、
         * @param slamtic_yaml 雷达yaml文件的地址
         * */
        LIDAR(std::string slamtic_yaml);

        /**
         * @brief 可以通过外部修改参数启动雷达
         * @param  lidar_uart_name 雷达的串口名称  baud_rate  雷达的波特率
         * */
        LIDAR(std::string lidar_uart_name , int baud_rate);
        /**
         * @brief 修改雷达的 串口名称
         * @param aimed_lidar_uart 雷达想要修改的串口名称
         * */

        void LIDAR_CHANGE_UART(std::string aimed_lidar_uart);

        /**
         * @brief 修改雷达的波特率
         * @param rate 修改后的雷达的比特率
         * */
        void LIDAR_CHANGE_BAUD_RATE(int rate);

        /**
         * @brief 雷达的启动部分
         * */
        void LIDAR_INIT();

        /**
         * @brief 设置雷达的扫描模式
         * */
        void LIDAR_SET_MODE();

        /**
         * @brief 雷达的关闭函数
         *
         * */
        void LIDAR_CLOSE();

        /**
         * @brief 雷达的捕获多线程
         * */
        static void *LIDAR_THREAD_CAPTURE(void *arg); //捕获线程
        /**
         * @brief 雷达的析构函数 作用是在雷达的类退出后 将雷达关闭
         * */
        ~LIDAR();
        /**
         * @brief 开启雷达的捕获线程
         * */
        void LIDAR_START_THREAD_CAPTURE();

    } ;

}