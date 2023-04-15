#include <slamtec/slamtec.h>


static s2::LIDAR* sl_lidar_ptr_=nullptr;



namespace s2{
    LIDAR::LIDAR(){

    }
    /**
     * @brief 通过读取雷达 yaml 文件 启动、
     * @param slamtic_yaml 雷达yaml文件的地址
     * */
    LIDAR::LIDAR(std::string slamtic_yaml){

        YAML::Node config = YAML::LoadFile(slamtic_yaml);

        this->lidar_uart_name = config["lidar_uart_name"].as<string>();
        cout << " lidar_uart_name " << this->lidar_uart_name<< endl;


        this->baud_rate = config["baud_rate"].as<int>();
        cout << " baud_rate " <<this->baud_rate<< endl;

    }

    /**
     * @brief 可以通过外部修改参数启动雷达
     * @param  lidar_uart_name 雷达的串口名称  baud_rate  雷达的波特率
     * */
    LIDAR::LIDAR(std::string lidar_uart_name , int baud_rate) {
        this ->lidar_uart_name = lidar_uart_name;
        this ->baud_rate = baud_rate;
    }

    /**
     * @brief 雷达的捕获多线程启动函数
     * */
    void LIDAR::LIDAR_START_THREAD_CAPTURE() {
        if(this->if_lidar_thread_capture_work){
            std::cout << "\033[32m Sl Lidar Capture Thread Is Working!" << std::endl;
        } else{
            this->if_lidar_thread_capture_work = true;
            pthread_create(&id_sl_lidar_capture,NULL,LIDAR_THREAD_CAPTURE,NULL);
        }
    }

    /**
     * @brief 雷达的捕获多线程
     * */
    void* s2::LIDAR::LIDAR_THREAD_CAPTURE(void  *arg) {
        int lidar_loss_time = 0; // 雷达丢失次数

        while (1){

            sl_lidar_response_measurement_node_hq_t lidar_nodes1[8192]; //雷达所扫描到的点

            size_t nodecount2=sizeof(lidar_nodes1)/sizeof(sl_lidar_response_measurement_node_hq_t);


            for(int i=0;i<8192;i++){
                sl_lidar_response_measurement_node_hq_t lidar_nodes_cover;
                lidar_nodes1[i]=lidar_nodes_cover;
            }
            auto res = sl_lidar_ptr_->lidar->grabScanDataHq(lidar_nodes1, nodecount2,60); // 捕获点云
            if (SL_IS_FAIL(res)){   //雷达信号丢失
                lidar_loss_time++;
                if(lidar_loss_time > 2)
                    printf("\033[31m lidar cannot capture\033[32m \n");

                if(lidar_loss_time > 50){
                    sl_lidar_ptr_->LIDAR_CLOSE();
                    usleep(10000);
                    sl_lidar_ptr_->LIDAR_INIT();
                    sl_lidar_ptr_->LIDAR_SET_MODE();
                    lidar_loss_time = 0;
                }
                continue;
            }
            else
            {
                lidar_loss_time=0;

            }


            pthread_mutex_lock(&sl_lidar_ptr_->mutex_sl_lidar_t);
            for(int i=0;i<8192;i++)
                sl_lidar_ptr_->lidar_nodes[i]=lidar_nodes1[i];
            pthread_mutex_unlock(&sl_lidar_ptr_->mutex_sl_lidar_t);
        }
    }


    /**
    * @brief 修改雷达的 串口名称
    * @param aimed_lidar_uart 雷达想要修改的串口名称
    * */
    void LIDAR::LIDAR_CHANGE_UART(std::string aimed_lidar_uart ){
        lidar_uart_name = aimed_lidar_uart;
        printf("\033[32m LIDAR UART CHANGE SUCCESSFULL");
        return ;
    }

    /**
     * @brief 修改雷达的波特率
     * @param rate 修改后的雷达的比特率
     * */
    void LIDAR::LIDAR_CHANGE_BAUD_RATE(int rate){
        baud_rate = rate;
        printf("\033[32m LIDAR RATE CHANGE sucessfully");
    }

    /**
     * @brief 设置雷达的扫描模式
     * */
    void LIDAR::LIDAR_SET_MODE() {
        std::vector<sl::LidarScanMode> scan_modes;
        this->lidar->getAllSupportedScanModes(scan_modes);
        this->lidar->startScanExpress(false, scan_modes[this->lidar_mode].id);
        std::cout << "\033[32m LIDAR SCAN MODE:" << scan_modes[this->lidar_mode].scan_mode << std::endl;
    }

    /**
     * @brief 雷达的启动部分
     * */
    void LIDAR::LIDAR_INIT() {
        /* 停止雷达 */
        sl::Result<sl::IChannel*> channel0= sl::createSerialPortChannel(this->lidar_uart_name, this->baud_rate);
        sl::ILidarDriver *lidar0 = *sl::createLidarDriver();

        auto res0 = lidar0->connect(*channel0);
        if(SL_IS_OK(res0)){
            sl_lidar_response_device_info_t deviceInfo0;
            res0 = lidar0->getDeviceInfo(deviceInfo0);
            if(SL_IS_OK(res0)){
                lidar0->reset();
            }
        }
        if(sl_lidar_ptr_== nullptr){
            sl_lidar_ptr_=this;
        }
        /* 创建通讯通道 */
        sl::IChannel* _channel;
        sl::Result<sl::IChannel*> channel= sl::createSerialPortChannel(this->lidar_uart_name, this->baud_rate);
        this->lidar = *sl::createLidarDriver();
        auto res = this->lidar->connect(*channel);
        if(SL_IS_OK(res)){
            sl_lidar_response_device_info_t deviceInfo;
            res = this->lidar->getDeviceInfo(deviceInfo);
            if(SL_IS_OK(res)){
                printf("\033[32m Model: %d, Firmware Version: %d.%d, Hardware Version: %d\n",
                       deviceInfo.model,
                       deviceInfo.firmware_version >> 8, deviceInfo.firmware_version & 0xffu,
                       deviceInfo.hardware_version);

                // check health
                sl_result op_result;
                sl_lidar_response_device_health_t healthinfo;

                op_result = this->lidar->getHealth(healthinfo);
                if(SL_IS_OK(op_result))
                {
                    printf("\033[32m SLAMTEC Lidar health status : %d\n", healthinfo.status);
                    if (healthinfo.status == SL_LIDAR_STATUS_WARNING)
                    {
                        printf("\033[33m slamtec lidar health warning !\n");
                    }
                    if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
                        fprintf(stderr, "\033[31m Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
                    }
                }
                else
                {
                    fprintf(stderr, "\033[31m Error, cannot retrieve the lidar health code: %x\n", op_result);
                }

                printf("\033[32m LIDAR INIT SUCCESS!\n");
            }else{
                fprintf(stderr, "\033[31m Failed to get device information from LIDAR %08x\r\n", res);
            }
        }else{
            fprintf(stderr, "\033[31 Failed to connect to LIDAR %08x\r\n", res);
        }
        this->LIDAR_SET_MODE();
        printf("\033[32 LIDAR SET SUCCESS!\n");
    }

    /**
     * @brief 雷达的关闭函数
     *
     * */
    void LIDAR::LIDAR_CLOSE() {
        this->lidar->stop();
        usleep(2000);
        this->lidar->setMotorSpeed(0);
        printf("\033[31 S2 LIDAR CLOSED!\n");
    }

    /**
     * @brief 雷达的析构函数 作用是在雷达的类退出后 将雷达关闭
     * */
    LIDAR::~LIDAR() {
        this->LIDAR_CLOSE();
    }
}