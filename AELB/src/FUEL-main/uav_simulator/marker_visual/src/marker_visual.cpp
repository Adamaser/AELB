#include <iostream>
#include <vector>
#include <string>
#include<iomanip>//必要头文件

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>
#include <Eigen/Geometry>


using namespace std ;

int makerIdCount  = 0 ;
visualization_msgs::MarkerArray    lane ;
visualization_msgs::MarkerArray    box ;
ros::Publisher marker_pub_lane  ;
ros::Publisher marker_pub_box ;

vector<vector<int>> colorMap = vector<vector<int>> (11, vector<int>(3)) = {
        {160,82,45}, {255,140,0},{0,255,0}, {220,20,60},
        {245,255,250}, {0,0,255},{255,0,255},{255,20,147},{64,224,208} , {255,255,0},{255,0,0 },
};

void visualLaneLine(visualization_msgs::MarkerArray  &lineMakerArray, vector<Eigen::Vector3d> pointsVec,  int id, bool colorUsed, double timeDelay){  
        visualization_msgs::Marker line ;
        line.lifetime = ros::Duration(timeDelay,0);               //  ros::Duration();   永远不会被删除
        line.type = visualization_msgs::Marker::LINE_STRIP;             //  形状为线
        line.action = visualization_msgs::Marker::ADD;
        line.header.frame_id =  "world" ;
        line.ns = "lane";
        line.scale.x = 0.80;

        if(colorUsed){          //  true 使用彩色 ，false 单调色
            line.color.r =   colorMap[id%10][0];            //  10种颜色选择
            line.color.g =  colorMap[id%10][1]; 
            line.color.b =  colorMap[id%10][2]; 
        }else{
            line.color.r = 160 ;  
            line.color.g  = 82 ;
            line.color.b  = 45 ;    // 红色
        }
        line.color.a =  1.0f ;
        line.pose.orientation.w = 1;
        line.id = id ;             //  线不同line ID号不能重复

        for (int i = 0; i < pointsVec.size() ; i ++)
        {
            geometry_msgs::Point p;
            p.x =  pointsVec[i].x() ;
            p.y =  pointsVec[i].y() ;
            p.z =  pointsVec[i].z() ;
            line.points.push_back(p);
        }
        lineMakerArray.markers.push_back(line);
        makerIdCount ++ ;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "marker visual");
	ros::NodeHandle n;

    marker_pub_lane = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_lane_vec", 10);
    marker_pub_box = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_box", 10);

    //  生成点  max :(10  20  10)  min(-10 -20 -10)
    Eigen::Vector3d max(10, 20, 10);
    Eigen::Vector3d min(-10,-20,-10);
    Eigen::Vector3d p0_b(min.x(), max.y(), max.z());
    Eigen::Vector3d p1_b(min.x(), min.y(), max.z());
    Eigen::Vector3d p2_b(min.x(), min.y(), min.z());
    Eigen::Vector3d p3_b(min.x(), max.y(), min.z());
    Eigen::Vector3d p4_b(max.x(), max.y(), min.z());
    Eigen::Vector3d p5_b(max.x(), min.y(), min.z());
    Eigen::Vector3d p6_b(max.x(), min.y(), max.z());
    Eigen::Vector3d p7_b(max.x(), max.y(), max.z());

    // 直线点
    vector<vector<Eigen::Vector3d>> linesVec(8) ;       //存储三条线
    linesVec[0].push_back({20,12,0});
    linesVec[0].push_back({-20,12,0});
    linesVec[1].push_back({20,4,0});
    linesVec[1].push_back({-20,4,0});
    linesVec[2].push_back({20,-4,0});
    linesVec[2].push_back({-20,-4,0});
    linesVec[3].push_back({20,-12,0});
    linesVec[3].push_back({-20,-12,0});

    linesVec[4].push_back({12,20,0});
    linesVec[4].push_back({12,-20,0});
    linesVec[5].push_back({4,20,0});
    linesVec[5].push_back({4,-20,0});
    linesVec[6].push_back({-4,20,0});
    linesVec[6].push_back({-4,-20,0});
    linesVec[7].push_back({-12,20,0});
    linesVec[7].push_back({-12,-20,0});


    //  矩形点
    vector<Eigen::Vector3d>  boxPointsVec ;
    boxPointsVec.push_back(p0_b);boxPointsVec.push_back(p3_b);boxPointsVec.push_back(p2_b);
    boxPointsVec.push_back(p5_b);boxPointsVec.push_back(p6_b);boxPointsVec.push_back(p1_b);  
    boxPointsVec.push_back(p0_b);boxPointsVec.push_back(p7_b);boxPointsVec.push_back(p4_b);
    boxPointsVec.push_back(p3_b);boxPointsVec.push_back(p4_b);boxPointsVec.push_back(p5_b);
    boxPointsVec.push_back(p6_b);boxPointsVec.push_back(p7_b);           

    ros::Rate loop_rate(10);

    //  可视化直线 
    for(int i = 0; i < linesVec.size(); i++ ){
            visualLaneLine(lane, linesVec[i], makerIdCount, true, 1000); 
    }
    //  可视化box
    visualLaneLine(box,boxPointsVec, makerIdCount, false, 1000); 

    while(ros::ok()){

    marker_pub_lane.publish(lane);
    marker_pub_box.publish(box);

    loop_rate.sleep();
    }
    return 0 ;
}

