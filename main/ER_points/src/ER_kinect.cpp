#include <ER/ER_kinect.h>

/**
 * @brief 获取中间六根柱子的点云图并且分类
 * @param depth_image 深度图
 * @return 中间六根柱子的点云图
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr TRANS_TO_CLOUD(cv::Mat depth_image)
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
void CAMERA1_GET_INFORMATION()
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
void COLOR_detect(cv::Mat color_image_ocv)
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

        for(int i=0;i<color_image_ocv.rows;i++)
        {
            if(min_cout_rows[i]==0)
                continue;
            float temp = (min_red_rows[i] - min_blue_rows[i]) / min_cout_rows[i];
            if(temp > 40) {
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
        cv::putText(color_image_ocv, label, cv::Point(label_left - 1, label_top - 2),cv::FONT_HERSHEY_SIMPLEX, 1,(0, 255, 0), 3);
    }
}
