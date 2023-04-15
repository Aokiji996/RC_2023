#include <slamtec/slamtec.h>
#include <stdio.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

const float Pi = 3.14159265358979;
int main(int argc, char **argv)
{
    std::string lidar_xml_position = "/home/rc/下载/RC_2023/external/sl_lidar_2/config/slamtic.yaml";
     s2::LIDAR lader(lidar_xml_position);
     lader.LIDAR_INIT();
    lader.LIDAR_START_THREAD_CAPTURE();

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("Viewer_PCL1"));

    //设置默认的坐标系/
    //设置固定的元素。红色是X轴，绿色是Y轴，蓝色是Z轴。

    viewer1->addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(20, 0, 0), "x");
    viewer1->addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 5, 0), "y");
    viewer1->addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 0, 2),"z");


    while(1){

         pcl::PointCloud<pcl::PointXYZ>::Ptr vis_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
         vis_cloud1->clear();
         //now.lidar_a=vehicle_rotation_angle(nodeCount,nodes);  //针对兔子机器人在发射过去的途中是否发生底盘旋转，并求出底盘旋转的角度；
         float angle_in_degrees;
         float distance_in_meters;

         int point_times=0;

         int cout1=0;
         for(int i=0;i<8192;i++)
             if(lader.lidar_nodes[i].dist_mm_q2!=0)
                 point_times++;

         int loss_points=0;

         for (size_t i = 0; i < point_times; i++) {

             // 获取该点的角度和距离
             angle_in_degrees = lader.lidar_nodes[i].angle_z_q14 * 90.f / (1 << 14) + 1.8;
             distance_in_meters = lader.lidar_nodes[i].dist_mm_q2 / 1000.f / (1 << 2);

             angle_in_degrees = angle_in_degrees * Pi / 180.0000;


             float x = cos(angle_in_degrees) * distance_in_meters;

             float y = sin(angle_in_degrees) * distance_in_meters;

             float angle=0;

             float distance01= sqrt((x)*(x)+(y*y));
             if(-x>0)
                 angle=acos((-y)/(distance01));
             else
                 angle=acos((y)/(distance01))+3.1415926;

             cout1++;

             pcl::PointXYZ p;
             p.x = -x, p.y = y, p.z = 0;

             vis_cloud1->points.push_back(p);

         }

         vis_cloud1->width = vis_cloud1->size();
         vis_cloud1->height = 1;
         vis_cloud1->is_dense = false;

        viewer1->addPointCloud(vis_cloud1,"cloud2");
        viewer1->spinOnce(3);
        viewer1->removePointCloud("cloud2");

     }
    return 0;
}
