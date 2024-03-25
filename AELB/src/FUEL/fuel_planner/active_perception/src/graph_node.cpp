#include <active_perception/graph_node.h>
#include <path_searching/astar2.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>

namespace fast_planner {
// Static data
double ViewNode::vm_;
double ViewNode::am_;
double ViewNode::yd_;
double ViewNode::ydd_;
double ViewNode::w_dir_;

double ViewNode::wl_;
double ViewNode::wy_;
double ViewNode::wp_;

shared_ptr<Astar> ViewNode::astar_;
shared_ptr<RayCaster> ViewNode::caster_;
shared_ptr<SDFMap> ViewNode::map_;

// Graph node for viewpoints planning
ViewNode::ViewNode(const Vector3d& p, const double& y) {
  pos_ = p;
  yaw_ = y;
  parent_ = nullptr;
  vel_.setZero();  // vel is zero by default, should be set explicitly
}

double ViewNode::costTo(const ViewNode::Ptr& node) {
  vector<Vector3d> path;
  double c = ViewNode::computeCost(pos_, node->pos_, yaw_, node->yaw_, vel_, yaw_dot_, path);
  // std::cout << "cost from " << id_ << " to " << node->id_ << " is: " << c << std::endl;
  return c;
}

//这个函数的主要目的是在给定的地图上搜索一条从起点到终点的安全路径。如果可以直接连接，则直接连接；否则，通过不断减小的分辨率使用 A* 算法来搜索路径。
/*
输入：起点与目标点
输出：返回值为A*路径长度，路径保存在path中
*/
double ViewNode::searchPath(const Vector3d& p1, const Vector3d& p2, vector<Vector3d>& path) {
  // Try connect two points with straight line
  bool safe = true;
  Vector3i idx;
  caster_->input(p1, p2);
  while (caster_->nextId(idx)) {
    if (map_->getInflateOccupancy(idx) == 1 || map_->getOccupancy(idx) == SDFMap::UNKNOWN ||
        !map_->isInBox(idx)) {
      safe = false;
      break;
    }
  }
  if (safe) {
    path = { p1, p2 };
    return (p1 - p2).norm();
  }
  // Search a path using decreasing resolution
  vector<double> res = { 0.4 };
  for (int k = 0; k < res.size(); ++k) {
    astar_->reset();
    astar_->setResolution(res[k]);
    if (astar_->search(p1, p2) == Astar::REACH_END) {
      path = astar_->getPath();
      return astar_->pathLength(path);
    }
  }
  // Use Astar early termination cost as an estimate
  path = { p1, p2 };
  return 1000;
}

/*计算P1到P2的代价函数*/
/*
传入参数：p1起点位置、p2终点位置、y1起点角度、y2终点角度、起点速度v1
*/
double ViewNode::computeCost(const Vector3d& p1, const Vector3d& p2, const double& y1, const double& y2,
                             const Vector3d& v1, const double& yd1, vector<Vector3d>& path) {
  // Cost of position change
  double pos_cost = ViewNode::searchPath(p1, p2, path) / vm_;

  // Consider velocity change
  if (v1.norm() > 1e-3) {
    Vector3d dir = (p2 - p1).normalized();
    Vector3d vdir = v1.normalized();
    double diff = acos(vdir.dot(dir));
    pos_cost += w_dir_ * diff;
    // double vc = v1.dot(dir);
    // pos_cost += w_dir_ * pow(vm_ - fabs(vc), 2) / (2 * vm_ * am_);
    // if (vc < 0)
    //   pos_cost += w_dir_ * 2 * fabs(vc) / am_;
  }

  // Cost of yaw change
  double diff = fabs(y2 - y1);
  diff = min(diff, 2 * M_PI - diff);
  double yaw_cost = diff / yd_;
  return max(pos_cost, yaw_cost);

  // // Consider yaw rate change
  // if (fabs(yd1) > 1e-3)
  // {
  //   double diff1 = y2 - y1;
  //   while (diff1 < -M_PI)
  //     diff1 += 2 * M_PI;
  //   while (diff1 > M_PI)
  //     diff1 -= 2 * M_PI;
  //   double diff2 = diff1 > 0 ? diff1 - 2 * M_PI : 2 * M_PI + diff1;
  // }
  // else
  // {
  // }
}

//计算内部TSP代价函数
double ViewNode::ComputeInternalCost(const Vector3d& p1, const Vector3d& p2, const double& y1, const double& y2,
                             const Vector3d& v1, const double& yd1, vector<Vector3d>& path,
                             const int& priority1, const int& priority2){

  //测试打印优先级
  std::cout << "当前坐标分别为" << '('<< p1(0)<<','<< p1(1)<<','<< p1(2)<<')' << "," <<  '('<< p2(0)<<','<< p2(1)<<','<< p2(2)<<')' << std::endl;
  std::cout << "当前优先级分别为" <<priority1 << "," << priority2 << std::endl;
  //求f_l:
  // Cost of position change
  double pos_cost = ViewNode::searchPath(p1, p2, path) / vm_;
  // Consider velocity change
  if (v1.norm() > 1e-3) {
    Vector3d dir = (p2 - p1).normalized();
    Vector3d vdir = v1.normalized();
    double diff = acos(vdir.dot(dir));
    pos_cost += w_dir_ * diff;
    // double vc = v1.dot(dir);
    // pos_cost += w_dir_ * pow(vm_ - fabs(vc), 2) / (2 * vm_ * am_);
    // if (vc < 0)
    //   pos_cost += w_dir_ * 2 * fabs(vc) / am_;
  }
  double diff = fabs(y2 - y1);
  diff = min(diff, 2 * M_PI - diff);
  double yaw_cost = diff / yd_;
  double f_l = max(pos_cost, yaw_cost);
  //求f_p
  double f_p = priority2;
  //求f_d
  double dx = p1(0)-p2(0);
  double dy = p1(1)-p2(1);
  double dz = p1(2)-p2(2);
  double f_dy = sqrt(dx*dx + dy*dy + dz*dz);
  return wl_ * f_l + wp_ * f_p + wy_ * f_dy;
  }

}
