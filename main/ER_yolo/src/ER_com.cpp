#include <ER/ER_com.h>

extern com::FRAME f;
extern com::USART u1;
extern float a[3];
extern bool if_change ;
extern char change[3];
/**
 * @brief ER数据滤波
 * @param results
 */
void CAMERA_SEND_INFORMATION_ER(std::vector<YOLO::yolo_output> &results){
    static int count=0;
    static int deteccte_if_fiter=0;

    count++;
    if(count >= 1){
        count = 0;
        float horizontal_angle[9] = {1.0};
        float distance[9] = {1.0};
        float height[9] = {1.0};
        for(size_t i = 0; i < results.size(); i++){
            //std::sort(send_data_map[i].horizontal_angle.begin(), send_data_map[i].horizontal_angle.end());
            horizontal_angle[results[i].index] =  results[i].y  ;

            //std::sort(send_data_map[i].distance.begin(), send_data_map[i].distance.end());
            distance[results[i].index] = results[i].x  ;

            //std::sort(send_data_map[i].height.begin(), send_data_map[i].height.end());
            height[results[i].index] = results[i].height ;
        }

        static char save_last_top_color[8]={'N','N','N','N','N','N','N','N'};

        /* for(int j=0;j<8;j++){
             if(results[j].tag == 'N'){
                 results[j].tag = save_last_top_color[j];
             }
         }
 */
        for(int i = 0; i < 8; i ++){
            if(!isnan(distance[i+1]) && i != 6 && i != 7) {
                f.FRAME_ADD_DATA('?');
                f.FRAME_ADD_DATA(i + 1);
                f.FRAME_ADD_DATA(distance[i+1]);
                f.FRAME_ADD_DATA(horizontal_angle[i+1]);
                f.FRAME_ADD_DATA(results[i].tag);
                /*超时检测*/
                usleep(0);


            }

            if(i == 6 || i == 7) {
                f.FRAME_ADD_DATA('?');
                f.FRAME_ADD_DATA(i + 1);
                f.FRAME_ADD_DATA(distance[i+1]);
                f.FRAME_ADD_DATA(horizontal_angle[i+1]);

                f.FRAME_ADD_DATA('B');
            }
        }
        f.FRAME_ADD_DATA('!');
        f.FRAME_SEND_DATA(u1);

    }
}


/* set receive callback */
void get_receive(char *data, int length, com::FRAME frame)
{
    if_change = (if_change + 1) % 2;
    int i=0,j=0;
    /* 展示如何从中获取对应的数据 */
    for(auto iter = frame.format.begin();iter != frame.format.end(); ++iter){
        switch (iter->second) {
            case com::is_char:
                change[i]=data[iter->first];
                std::cout << change[i] << ' ';
                i++;
                break;
            case com::is_int:
                int c;
                c = data[iter->first] - '0';
                std::cout << c << ' ';
                break;
            case com::is_float:

                a[j] = *(float *)(&data[iter->first]);
                std::cout << a[j] << ' ';
                j++;
                break;
            case com::is_double:
                double b;
                b = *(double *)(&data[iter->first]);
                std::cout << b << ' ';
                break;
        }
    }
    if (change[0] == 'c') {
        if_change = (if_change + 1) % 2;
    }
    std::cout << std::endl;
}
