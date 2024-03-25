#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

using Eigen::Vector3d;
using std::vector;
using std::shared_ptr;
using std::unique_ptr;
using std::string;

namespace fast_planner {
class FastPlannerManager;
class FastExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

/*枚举状态机类型*/
enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH };

class FastExplorationFSM {
private:
  /* planning utils */
  /*对FastPlannerManager类实例的共享指针。*/
  shared_ptr<FastPlannerManager> planner_manager_;
  /*对FastExplorationManager类实例的共享指针。*/
  shared_ptr<FastExplorationManager> expl_manager_;
  /*PlanningVisualization类实例的共享指针。*/
  shared_ptr<PlanningVisualization> visualization_;
  /*
  fp_：对FSMParam结构体实例的共享指针。
  fd_：对FSMData结构体实例的共享指针。
  state_：表示FSM当前状态的枚举（EXPL_STATE）。
  classic_：一个布尔值，指示勘探是否处于经典模式。
  */
  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  bool classic_;

/*
  node_：用于与ROS系统进行交互的ROS NodeHandle。
  exec_timer_：用于执行规划和勘探任务的ROS定时器。
  safety_timer_：用于与安全相关任务的ROS定时器。
  vis_timer_：用于可视化的ROS定时器。
  trigger_sub_：用于接收触发信号的ROS订阅者。
  odom_sub_：用于接收里程计数据的ROS订阅者。
  replan_pub_：用于发布重新规划信息的ROS发布者。
  new_pub_：用于发布新信息的ROS发布者。
  bspline_pub_：用于发布B样条曲线信息的ROS发布者。
*/
  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;

  /* helper functions */
  /*调用勘探规划器的辅助函数*/
  int callExplorationPlanner();
  /*转换FSM状态的辅助函数。*/
  void transitState(EXPL_STATE new_state, string pos_call);

  /* ROS functions */
  /*
  FSMCallback()：FSM的回调函数，由ROS定时器触发。
  safetyCallback()：安全定时器的回调函数。
  frontierCallback()：勘探前沿定时器的回调函数。
  triggerCallback()：接收触发信号的回调函数。
  odometryCallback()：接收里程计数据的回调函数。
  visualize()：可视化函数。
  clearVisMarker()：清除可视化标记的函数。
  */
  void FSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);
  void triggerCallback(const nav_msgs::PathConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void visualize();
  void clearVisMarker();

public:
  FastExplorationFSM(/* args */) {
  }
  ~FastExplorationFSM() {
  }

  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif