#include <glog/logging.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>
#include "omp.h"
#include <string>
#include <algorithm>
#include <unordered_map>
#include <map>
#include <cmath>
#include <iostream>
#include "mypcl_cloud_type.h"
#include "voxelMap.hpp"
#include "so3_math.h"

using namespace std;
using namespace Eigen;

string lidar_topic;
double leaf_size = 0.2;
Eigen::Vector3d ext;


ros::Publisher pub_deskew;
// void sub_sample(RTVPointCloud::Ptr &input_points, double size_voxel, double range)
// {
//     unordered_map<VOXEL_LOC, vector<RTVPoint>> grid_map;
//     for (uint i = 0; i < input_points->size(); i++)
//     {
//         if (sqrt(input_points->points[i].x * input_points->points[i].x + input_points->points[i].y * input_points->points[i].y + input_points->points[i].z * input_points->points[i].z) < 150)
//         {
//             float loc_xyz[3];
//             loc_xyz[0] = input_points->points[i].x / size_voxel;
//             loc_xyz[1] = input_points->points[i].y / size_voxel;
//             loc_xyz[2] = input_points->points[i].z / size_voxel;
//             for (int j = 0; j < 3; j++)
//             {
//                 if (loc_xyz[j] < 0)
//                 {
//                     loc_xyz[j] -= 1.0;
//                 }
//             }
//             VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
//             grid_map[position].push_back(input_points->points[i]);
//         }
//     }

//     input_points->resize(0);
//     for (const auto &n : grid_map)
//     {
//         if (n.second.size() > 0)
//         {
//             input_points->points.push_back(n.second[0]);
//         }
//     }
// }

void pclCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg){
    
    RTVPointCloud::Ptr ptr(new RTVPointCloud());
    pcl::fromROSMsg(*msg, *ptr);
    double end_time = ptr->points.back().time;
    for (int i = 0; i < ptr->points.size(); i++)
    {
        double dt = end_time - ptr->points[i].time;
        Eigen::Vector3d point = Eigen::Vector3d(ptr->points[i].x,ptr->points[i].y,ptr->points[i].z);
        Eigen::Vector3d linear_velocity = ptr->points[i].velocity * point / point.norm();
        double w_z = linear_velocity.y() / ext.x() * 0.01;
        double w_y = linear_velocity.z() / ext.z() * 0.01;

        Eigen::Vector3d angular_velocity = Eigen::Vector3d(0, w_y, -w_z);
        Eigen::Matrix3d rot = Exp(angular_velocity, dt);
        Eigen::Vector3d aft_point = rot * point + linear_velocity * dt;

        ptr->points[i].x = aft_point.x();
        ptr->points[i].y = aft_point.y();
        ptr->points[i].z = aft_point.z();
    }
    sensor_msgs::PointCloud2 pub_cloud;
    pcl::toROSMsg(*ptr, pub_cloud);
    pub_cloud.header = msg->header;
    pub_deskew.publish(pub_cloud);


}


int main(int argc, char **argv){
    ros::init(argc, argv, "deskew");
    ros::NodeHandle nh;
    std::vector<double> extrinT;
    nh.param<vector<double>>("extrinsic_T_v_l", extrinT, vector<double>());
    ext << extrinT[0], extrinT[1], extrinT[2];

    ros::Subscriber sub_pcl = nh.subscribe("/aeva_points", 2000, pclCallBack);
    pub_deskew = nh.advertise<sensor_msgs::PointCloud2>("/deskew_points", 10);
    ros::spin();
    return 0;
}
