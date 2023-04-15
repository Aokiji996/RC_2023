#include <ER/ER_lidar.h>

const float Pi = 3.14159265358979;

float cal_dis(float x1, float x2, float y1, float y2){
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LIDAR_LOCATION_TARGETING(std::vector<YOLO::yolo_output> &results , s2::LIDAR *lidar)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr vis_cloud1(new pcl::PointCloud<pcl::PointXYZ>);

    //now.lidar_a=vehicle_rotation_angle(nodeCount,nodes);  //针对兔子机器人在发射过去的途中是否发生底盘旋转，并求出底盘旋转的角度；
    float angle_in_degrees;
    float distance_in_meters;
    std::queue<point_pose> civer;
    points=civer;

    int node_sizes=0;

    for (size_t i = 0; i < lidar->node_count; i++) {

        // 获取该点的角度和距离
        angle_in_degrees =lidar->lidar_nodes[i].angle_z_q14 * 90.f / (1 << 14) + 1.8;
        distance_in_meters = lidar->lidar_nodes[i].dist_mm_q2 / 1000.f / (1 << 2);

        angle_in_degrees=angle_in_degrees - 1;
        if (abs(distance_in_meters) < 0.5) continue;
        if(abs(distance_in_meters)>6) continue;

        angle_in_degrees = angle_in_degrees * Pi / 180.0000;

//
//            //将雷达测得的角度与实际角度向校准，得到雷达实际测得的柱的角度

        float x =- cos(angle_in_degrees) * distance_in_meters;

        float y = -sin(angle_in_degrees) * distance_in_meters;

        if(x<0)
            continue;

        float  angle=0;
        angle=acos((y)/(distance_in_meters));
        points.push({distance_in_meters, angle,angle_in_degrees,x,-y,0});

        pcl::PointXYZ p;
        p.x = x, p.y = -y, p.z = 0;

        vis_cloud1->points.push_back(p);

    }
    vis_cloud1->width = vis_cloud1->size();
    vis_cloud1->height = 1;
    vis_cloud1->is_dense = false;

    if(points.size() == 0)
        return vis_cloud1;
    Sort_Point();
    Select_Colum_Information();
    select_colum();
    ER_compute_the_real_colum();


    for(int i=0;i<2;i++){


        YOLO::yolo_output temper;

        //  std::cout<<"  colum_x "<<colums[i].x<<endl;
        //  std::cout<<"  colum_y "<<colums[i].y<<endl;

        if((!isnan(colums[i].x))||(!isnan(colums[i].y)))
        {
            temper.y=colums[i].x;
            temper.x=colums[i].y;

        } else
        {
            temper.y=0.00001f;
            temper.x=0.00001f;
        }
        temper.distance=colums[i].distance;
        temper.horizontal_angle=(colums[i].angle);
        temper.index=colums[i].index;
        results.push_back(temper);

    }

    return vis_cloud1;
}

void Sort_Point(){

    for(int i = 0; i < 200; i ++){
        sort_points[i].clear();
    }

    int idx = 0;
    for(int i = 0; i < points.size() - 1; ){
        point_pose temp_point_1 = points.front();
        points.pop();
        point_pose temp_point_2 = points.front();
        if(cal_dis(temp_point_1.x, temp_point_2.x, temp_point_1.y, temp_point_2.y) < 0.3)
            sort_points[idx].push_back(temp_point_1);
        else{
            sort_points[idx].push_back(temp_point_1);
            idx ++;
            continue;
        }
    }
}



void Select_Colum_Information()
{
    point_pose tempmentmin;
    int nums=0;

    for(int i=0;i<100;i++){
        colum_information[i]=tempmentmin;
    }

    //point_pose tempment_return,tempment_receve;

    for(int i=0;(sort_points[i].size()!=0)&&(i<100);i++) {
        nums = sort_points[i].size();

        tempmentmin=sort_points[i][0];
        for(int j=1;(j<100)&&(j<sort_points[i].size());j++){
            if(tempmentmin.distance>sort_points[i][j].distance)
                tempmentmin=sort_points[i][j];
        }
        tempmentmin.select_colum_times=i;
        tempmentmin.point_nums=nums;
        colum_information[i]=tempmentmin;
    }


    return;
}

void select_colum()
{
    colums[0].index=7;
    colums[1].index=8;

    for(int i=0;i<=50;i++)
    {
        float x=colum_information[i].x;
        float y=colum_information[i].y;


        if((x>1.0)&&(x<2.3)) {
            if ((y > 2.5) && (y < 5.7)){
                o7.x=x;
                o7.y=y;

                colum_information[i].index = 7;
                colums[0]=colum_information[i];

            }

            else if ((y<-1.5)&&(y>-5.0)){
                o8.x=x;
                o8.y=y;
                colum_information[i].index=8;
                colums[1]=colum_information[i];
            }

        }
    }
    return ;
}


/**
 * @brief 针对ER车 雷达 摆放不齐而对其进行处理，其主要操作就是对通过三角型来获取以三角形为准的 7号 和 8号 的位置
 *
 * */
void ER_compute_the_real_colum(){

    float colum_distance[2]={0,0},colum_angle[2]={0,0};

    for(int i=0;i<2;i++){

        for(int j=0;j<colums[i].point_nums;j++){
            colum_distance[i] = colum_distance[i]+sort_points[colums[i].select_colum_times][j].distance;
            colum_angle[i] = colum_angle[i]+sort_points[colums[i].select_colum_times][j].angle;
        }


        if(colums[i].point_nums==0){
            colum_distance[i]=0;
            colum_angle[i]=0;
        } else {
            colum_distance[i] = colum_distance[i] / colums[i].point_nums;
            colum_angle[i] = colum_angle[i] / colums[i].point_nums;
        }
    }

    float angle_seven_and_eight=0;

    angle_seven_and_eight= fabs(colum_angle[0]-colum_angle[1]);

    float three_l;
    three_l= sqrt(colum_distance[0] * colum_distance[0] + colum_distance[1] * colum_distance[1] - 2 * colum_distance[0] * colum_distance[1]*
                                                                                                  cos(angle_seven_and_eight));

    float shu,heng[2];
    shu=(colum_distance[0] * colum_distance[1] * sin(angle_seven_and_eight))/three_l ;

    heng[0] = sqrt(colum_distance[0] * colum_distance[0] - shu * shu);
    heng[1] = sqrt(colum_distance[1] * colum_distance[1] - shu *shu );


    colums[0].x=shu-0.03-0.03;
    colums[0].y =- heng[0]-0.41 + 0.025 + 0.015;
    colums[1].x=shu-0.03 - 0.03;
    colums[1].y=heng[1]-0.41 +0.025 + 0.015;
    return;

}