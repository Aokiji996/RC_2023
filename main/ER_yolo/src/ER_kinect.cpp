#include <ER/ER_kinect.h>

void quicksort_x(std::vector<YOLO::yolo_output> &q, int l, int r){
    if(l >= r)
        return;
    int mid = (l + r) / 2;
    float x = q[mid].x, i = l - 1, j = r + 1;
    while(i < j){
        do{
            i ++;
        }while(q[i].x < x);
        do{
            j --;
        }while(q[j].x > x);
        if(i < j){
            YOLO::yolo_output t = q[i];
            q[i] = q[j];
            q[j] = t;
        }
    }
    quicksort_x(q, l, j);
    quicksort_x(q, j + 1, r);
}

void quicksort_distance(std::vector<YOLO::yolo_output> &q, int l, int r){
    if(l >= r)
        return;
    int mid = (l + r) / 2;
//    float x = q[mid].distance, i = l - 1, j = r + 1;
    float x = fabs((q[mid].x *  q[0].x) + (q[mid].y * q[0].y)),
            i = l - 1, j = r + 1;
    while(i < j){
        do{
            i ++;
        }while( fabs( (q[i].x * q[0].x) + (q[i].y * q[0].y)) < x);
        do{
            j --;
        }while(  fabs( ( q[j].x * q[0].x ) + (q[j].y * q[0].y)) > x);
        if(i < j){
            YOLO::yolo_output t = q[i];
            q[i] = q[j];
            q[j] = t;
        }
    }
    quicksort_distance(q, l, j);
    quicksort_distance(q, j + 1, r);
}

/**
   * @brief 获取神经网络识别到的结果的所需信息
   * @param results 神经网络获取到的结果
   * @param point_cloud 相机捕获到的点云
   */
void CAMERA1_GET_INFORMATION(std::vector<YOLO::yolo_output> &results, cv::Mat depth_image_ocv , cv::Mat color_image_ocv){
    if(results.size() == 0){
        return;
    }
    for(int i = 0; i < results.size(); i ++){
        /*选取框的中心点作为目标点*/
        int x = (results[i].kpts.kpt[0].x + results[i].kpts.kpt[1].x + results[i].kpts.kpt[2].x + results[i].kpts.kpt[3].x + results[i].kpts.kpt[4].x + results[i].kpts.kpt[5].x)/6;
        int y = (results[i].kpts.kpt[0].y + results[i].kpts.kpt[1].y + results[i].kpts.kpt[2].y + results[i].kpts.kpt[3].y + results[i].kpts.kpt[4].y + results[i].kpts.kpt[5].y)/6;
        /*获取目标点的坐标*/
        float x_average = 0.0;
        float y_average = 0.0;
        float z_average =0.0;

        int times = 0;


        int k= x;
        int j= y;
        ushort d = depth_image_ocv.at<ushort>(j, k);
        float z1 = (float) d;
        z1 /= 1000.f;
        float x1 = (x - intrinsic_parameter[0].cx) * z1 / intrinsic_parameter[0].fx;
        float y1 = (y - intrinsic_parameter[0].cy) * z1 / intrinsic_parameter[0].fy;

        x_average += x1;
        y_average += y1;
        z_average += z1;

//        if((results[i].kpts.kpt[2].x+2 < results[i].image.cols)&&(results[i].kpts.kpt[3].x-2 > 0))
//        for(int k=results[i].kpts.kpt[0].y+2;k<results[i].kpts.kpt[2].y-2;k++) {
//            for (int j = results[i].kpts.kpt[0].x + 2; j < results[i].kpts.kpt[1].x - 2; j++) {
//                if ((j < 0) || (k < 0) || (k >= color_image_ocv.rows) || (j >= color_image_ocv.cols))
//                    continue;
//                ushort d = depth_image_ocv.at<ushort>(k, j);
//                float z1 = (float) d;
//                z1 /= 1000.f;
//                float x1 = (x - intrinsic_parameter[0].cx) * z1 / intrinsic_parameter[0].fx;
//                float y1 = (y - intrinsic_parameter[0].cy) * z1 / intrinsic_parameter[0].fy;
//                if (!isnan(x1) && !isnan(y1) && !isnan(z1) && x1 != 0 && y1 != 0 && z1 != 0) {
//                    x_average += x1;
//                    y_average += y1;
//                    z_average += z1;
//                    times++;
//                }
//            }
//        }
//        if(times!=0){
//            x_average = x_average / times;
//            y_average = y_average / times;
//            z_average = z_average / times;
//        }


        if(!isnan(x_average) && !isnan(y_average) && !isnan(z_average) && x_average!= 0 && y_average != 0 && z_average != 0) {
            results[i].height = -y_average;   /*得到目标物体的高度*/
//            results[i].height = -y1;
            results[i].distance = sqrt(powf(x_average, 2) + powf(z_average, 2));/*得到目标物体的距相机左摄像头的距离*/
            results[i].x = x_average;              /*得到相机的x，y坐标，为区分柱子*/
            results[i].y = abs(z_average);
            results[i].horizontal_angle = acos(float(results[i].x / sqrt(powf(results[i].x,2) + powf(results[i].y, 2))));
            results[i].image = color_image_ocv;
//                if (results[i].distance != 0) {
//                    results[i].elevation_angle = acos(float(results[i].y / results[i].distance));
//                }
        }else{
            results.erase(results.begin() + i);
            i --;
        }
    }
}

bool  CAMERA_DISTINGUISH_CYLINDER_ER(std::vector<YOLO::yolo_output> &results, int &b_diff_r, int &r_diff_g, int &temp_b, int &temp_r)
{
    int max_height_index = 0, min_height_index = 0;
    YOLO::yolo_output exchange_1, exchange_2;

    /*先将高度最高的柱子标号为1*/
    for(int i = 0; i < results.size(); i++)
    {
        if(results[i].distance > 3.0 && results[i].distance < 7.0)
            if (results[i].height > results[max_height_index].height)
                max_height_index = i;
        if (i == results.size() - 1)
        {
            results[max_height_index].index = 1;
            exchange_1 = results[max_height_index];
            results[max_height_index] = results[0];
            results[0] = exchange_1;
        }
    }

    // 标出 6 号柱
    for(int i = 1; i < results.size(); i++)
    {
        if(results[i].distance < 3.0)
            min_height_index = i;
        if(i == results.size() - 1)
        {
            results[min_height_index].index = 6;
            exchange_2 = results[min_height_index];
            results[min_height_index] = results[results.size() - 1];
            results[results.size() - 1] = exchange_2;
        }
    }

    quicksort_distance(results, 1, results.size() - 2);  //将除了六号柱 和一号柱 的柱子进行排序


    int size_infront_of_first = 0; //确定一号 前面有多少个柱子 主要是 对一号柱前面只识别到 1 个柱子的情况 并对其进行判断

    float one_to_one = fabs( results[0].x * results[0].x + results[0].y * results[0].y);   // 在一号柱 向量的 方向上的距离
    float six_to_one =  fabs( results[results.size() - 1].x * results[0].x + results[results.size() - 1].y * results[0].y);  //  6 号柱在一号柱的方向上的 投影
    for(int i = 1; i <= results.size() - 2; i++)
    {
        float i_0 = fabs(results[i].x * results[0].x + results[i].y * results[0].y);
        if((i_0 < one_to_one) && (i_0 > six_to_one))
        {
            size_infront_of_first++;
        }
    }

    if(size_infront_of_first == 1)  //如果 一号柱 前面只有一个柱子 在这中情况下  对前面的柱子进行判断
    {
        if(results[1].x > results[0].x)
            results[1].index = 3;
        else
            results[1].index = 2;

        quicksort_x(results,2,results.size() - 2);  //将除了 1  6  2 或3 号柱 进行 按x 排序 其目的是 排除后部的对面 6 号柱干扰

        if(results[2].distance < 8)  //目的是 避免 将后部6 号柱 误识别
        {
            if(fabs(results[2].x - results[0].x) > 1) //目的是避免将后面 6  号柱误识别
                results[2].index = 4;
            for(int i = 3; i <= results.size() - 2; i++)
            {
                if(results[i].distance > 8)  //目的是避免将后面 6  号柱误识别
                    continue ;
                if(fabs(results[i].x - results[0].x) > 1) //目的是避免将后面 6  号柱误识别
                    results[i].index = 5;
            }
        }
        else
        {
            if(fabs(results[3].x - results[0].x) > 1)
            results[3].index = 4;  //目的是避免将后面 6  号柱误识别 此时

            for(int i = 4; i <= results.size() - 2; i++)
            {
                if(results[i].distance > 8)
                    continue;
                if(fabs(results[i].x - results[0].x) > 1)
                    results[i].index = 5;
            }
        }
    }
    else if(size_infront_of_first == 2)  //如果前面有 2个柱子
    {
        // 通过将 坐标投影到 一号的方向来区分 2  3 号柱子
        if((fabs(results[1].x - results[2].x) > 1) && ( results[1].x * results[0].x + results[1].y * results[0].y <
                                                       pow(results[0].x ,2) + pow(results[0].y,2))   &&  (results[2].x * results[0].x + results[2].y * results[0].y <
                                                                                                          pow(results[0].x ,2) + pow(results[0].y,2)) )
        {
            if (results[1].x > results[2].x)
            {
                results[1].index = 3;
                results[2].index = 2;
                YOLO::yolo_output exchange = results[1];
                results[1] = results[2];
                results[2] = exchange;
            }
            else
            {
                results[1].index = 2;
                results[2].index = 3;
            }

        }
        //同理

        quicksort_x(results, 3, results.size() - 2);
        if (fabs(results[3].x - results[0].x) > 1)
            results[3].index = 4;

        float largest_x = fabs(results[4].x - results[0].x);
        int largets_size = 4;
        for (int i = 4; i <= results.size() - 2; i++)
        {

            if (fabs(results[i].x - results[0].x) > largest_x)
            {
                largets_size = i;
                largest_x = fabs(results[i].x - results[0].x);
            }
        }
        if (largest_x > 1)
            results[largets_size].index = 5;
    }

    for(int i = 0; i < results.size(); i++)
    {
        int width = (results[i].kpts.kpt[1].x - results[i].kpts.kpt[0].x) * 0.75;
        int height = results[i].kpts.kpt[2].y - results[i].kpts.kpt[0].y + 10;
        cv::Rect rect(results[i].kpts.kpt[0].x + 0.4 * width, results[i].kpts.kpt[0].y, width * 0.8, height);// 划定区域
        if (rect.x + width <= results[i].image.cols && rect.y + height <= results[i].image.rows)
        {
            results[i].roi = results[i].image(rect);
            int sum_b = 0, sum_r = 0, sum_g = 0;
            for(int k = 0; k < results[i].roi.rows; k++)
            {
                for(int j = 0; j < results[i].roi.cols; j++)
                {
                    /* 分别为各个通道的值 */
                    int b = results[i].roi.at<cv::Vec3b>(k, j)[0];
                    int g = results[i].roi.at<cv::Vec3b>(k, j)[1];
                    int r = results[i].roi.at<cv::Vec3b>(k, j)[2];
                    sum_b += b;
                    sum_g += g;
                    sum_r += r;

                    // if (b - g > 35) {
                    if (-675*r-286*g+665*b>5542)
                    {
                        results[i].roi.at<cv::Vec3b>(k, j)[0] = 255;
                        results[i].roi.at<cv::Vec3b>(k, j)[1] = 0;
                        results[i].roi.at<cv::Vec3b>(k, j)[2] = 0;
                        //  } else if (r - g > 80) {
                    }
                    else if (((4577*r -2339 * g +964 *b > 620000)&& ( -2186.17 * r +6087 *g - 5755.68 *b < -178973.39)&&(-109.92 * r + 1536.96 *g +1511.56 * b <500000))
                                ||((2485.5 * r - 5237.5 * g + 363 *b >0) &&(22*r-27*g+11*b  > 1305)))
                    {
                        results[i].roi.at<cv::Vec3b>(k, j)[0] = 0;
                        results[i].roi.at<cv::Vec3b>(k, j)[1] = 0;
                        results[i].roi.at<cv::Vec3b>(k, j)[2] = 255;
                    }
                    else
                    {
                        results[i].roi.at<cv::Vec3b>(k, j)[0] = 0;
                        results[i].roi.at<cv::Vec3b>(k, j)[1] = 0;
                        results[i].roi.at<cv::Vec3b>(k, j)[2] = 0;
                    }
                }
            }
            sum_b /= (results[i].roi.cols * results[i].roi.rows);
            sum_g /= (results[i].roi.cols * results[i].roi.rows);
            sum_r /= (results[i].roi.cols * results[i].roi.rows);

            for(int k = 0;k < results[i].roi.rows;k++)
            {
                int r_sum=0, b_sum=0;
                for(int j = 0;j < results[i].roi.cols;j++)
                {
                    /* 分别为各个通道的值 */
                    int b = results[i].roi.at<cv::Vec3b>(k, j)[0];
                    int g = results[i].roi.at<cv::Vec3b>(k, j)[1];
                    int r = results[i].roi.at<cv::Vec3b>(k, j)[2];
                    r_sum += r;
                    b_sum += b;
                }
                float temp = (r_sum - b_sum)/results[i].roi.cols;
                if(temp > 40)
                {
                    results[i].tag = 'R';
                    break;
                }
                else if(temp < -120)
                {
                    results[i].tag = 'B';
                    break;
                }
            }
        }
    }
    return true;
}
