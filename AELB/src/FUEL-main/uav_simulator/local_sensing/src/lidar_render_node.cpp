#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <pcl/search/impl/kdtree.hpp>
#include <vector>

using namespace std;
using namespace Eigen;

// publisher cloud and pose of lidar
ros::Publisher pub_lidar_cloud, pub_liadr_pose;

//subscriber odo and map
ros::Subscriber odom_sub;
ros::Subscriber global_map_sub, local_map_sub;

sensor_msgs::PointCloud2 local_map_pcl;
sensor_msgs::PointCloud2 local_depth_pcl;


ros::Timer local_sensing_timer, pose_timer;

bool has_global_map(false);
bool has_local_map(false);
bool has_odom(false);

nav_msgs::Odometry odom_;
Eigen::Matrix4d sensor2body, sensor2world;

double sensing_horizon, sensing_rate, estimation_rate;
double x_size, y_size, z_size;
double gl_xl, gl_yl, gl_zl;
double resolution, inv_resolution;
int GLX_SIZE, GLY_SIZE, GLZ_SIZE;

ros::Time last_odom_stamp = ros::TIME_MAX;

pcl::PointCloud<pcl::PointXYZ> cloud_all_map, local_map;
pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
sensor_msgs::PointCloud2 local_map_pcd;

pcl::search::KdTree<pcl::PointXYZ> _kdtreeLocalMap;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i& index)
{
  Eigen::Vector3d pt;
  pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
  pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
  pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

  return pt;
};

inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d& pt)
{
  Eigen::Vector3i idx;
  idx(0) = std::min(std::max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1);
  idx(1) = std::min(std::max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1);
  idx(2) = std::min(std::max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

  return idx;
};

//根据里程计信息动态更新传感器到全局坐标系的变换矩阵
void rcvOdometryCallbck(const nav_msgs::Odometry& odom)
{
  /*if(!has_global_map)
    return;*/
  has_odom = true;
  odom_ = odom;

  Matrix4d body2world = Matrix4d::Identity();

  Eigen::Vector3d request_position;
  Eigen::Quaterniond pose;
  pose.x() = odom.pose.pose.orientation.x;
  pose.y() = odom.pose.pose.orientation.y;
  pose.z() = odom.pose.pose.orientation.z;
  pose.w() = odom.pose.pose.orientation.w;
  body2world.block<3, 3>(0, 0) = pose.toRotationMatrix();
  body2world(0, 3) = odom.pose.pose.position.x;
  body2world(1, 3) = odom.pose.pose.position.y;
  body2world(2, 3) = odom.pose.pose.position.z;

  // convert to cam pose
  sensor2world = body2world * sensor2body;
}

//处理全局点云信息回调函数
void rcvGlobalPointCloudCallBack(const sensor_msgs::PointCloud2& pointcloud_map)
{
  //有全局点云信息时候避免重复处理
  if (has_global_map)
    return;

  ROS_WARN("Global Pointcloud received..");
  //创建PCL点云对象存储ros中转换好的点云信息
  pcl::PointCloud<pcl::PointXYZ> cloud_input;
  pcl::fromROSMsg(pointcloud_map, cloud_input);
  //设置体素格子的尺寸，这是一个体素滤波器，用于对点云进行下采样。
  _voxel_sampler.setLeafSize(0.1f, 0.1f, 0.1f);
  _voxel_sampler.setInputCloud(cloud_input.makeShared());
  _voxel_sampler.filter(cloud_all_map);
  //加入KD树，加速搜索附近点
  _kdtreeLocalMap.setInputCloud(cloud_all_map.makeShared());

  has_global_map = true;
}

//发布传感器全局坐标信息
void pubSensorPose(const ros::TimerEvent& e)
{
  Eigen::Quaterniond q;
  //从 sensor2world 矩阵中提取旋转部分，将其转换为四元数 q。这个四元数表示传感器在世界坐标系中的姿态。
  q = sensor2world.block<3, 3>(0, 0);

  geometry_msgs::PoseStamped sensor_pose;
  sensor_pose.header = odom_.header;
  sensor_pose.header.frame_id = "/map";
  sensor_pose.pose.position.x = sensor2world(0, 3);
  sensor_pose.pose.position.y = sensor2world(1, 3);
  sensor_pose.pose.position.z = sensor2world(2, 3);
  sensor_pose.pose.orientation.w = q.w();
  sensor_pose.pose.orientation.x = q.x();
  sensor_pose.pose.orientation.y = q.y();
  sensor_pose.pose.orientation.z = q.z();
  pub_liadr_pose.publish(sensor_pose);
}

//检测lidar区域内的点云信息并发布
void renderLidarSensedPoints(){
    
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pcl_lidar_render");
    ros::NodeHandle nh("~");

    // subscribe global map point cloud
    global_map_sub = nh.subscribe("global_map", 1, rcvGlobalPointCloudCallBack);
    odom_sub = nh.subscribe("odometry", 50, rcvOdometryCallbck);

    //publish lidar's point_cloud in FOV and lidar's pose
    pub_lidar_cloud = nh.advertise<sensor_msgs::PointCloud2>("/pcl_lidar_render/lidar_cloud", 10);
    pub_liadr_pose = nh.advertise<geometry_msgs::PoseStamped>("/pcl_lidar_render/lidar_pose", 10);
    double lidar_sensing_duration = 1.0 / sensing_rate;
    double lidar_estimate_duration = 1.0 / estimation_rate;

    //timer trigger
    local_sensing_timer = nh.createTimer(ros::Duration(lidar_sensing_duration), renderLidarSensedPoints);
    pose_timer = nh.createTimer(ros::Duration(lidar_estimate_duration), pubSensorPose);

    inv_resolution = 1.0 / resolution;
    gl_xl = -x_size / 2.0;
    gl_yl = -y_size / 2.0;
    gl_zl = 0.0;
    GLX_SIZE = (int)(x_size * inv_resolution);
    GLY_SIZE = (int)(y_size * inv_resolution);
    GLZ_SIZE = (int)(z_size * inv_resolution);

    //lidar to body
    sensor2body << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    ros::Rate rate(100);
    bool status = ros::ok();
    while (status)
    {
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }
}
