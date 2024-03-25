#ifndef _FRONTIER_FINDER_H_
#define _FRONTIER_FINDER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <list>
#include <utility>

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;
using std::list;
using std::pair;

class RayCaster;

namespace fast_planner {
class EDTEnvironment;
class PerceptionUtils;

// Viewpoint to cover a frontier cluster
struct Viewpoint {
  // Position and heading
  Vector3d pos_;
  double yaw_;
  // Fraction of the cluster that can be covered
  // double fraction_;
  int visib_num_;
};

// A frontier cluster, the viewpoints to cover it
//定义了论文中的数据结构
struct Frontier {
  // Complete voxels belonging to the cluster
  vector<Vector3d> cells_;//所有该边界的元素
  // down-sampled voxels filtered by voxel grid filter
  vector<Vector3d> filtered_cells_;//由体素网格过滤后的下采样体素
  // Average position of all voxels
  Vector3d average_;//所有边界元素平均位置
  // Idx of cluster
  int id_;//当前边界ID
  // Viewpoints that can cover the cluster
  vector<Viewpoint> viewpoints_;//视点集合
  // Bounding box of cluster, center & 1/2 side length
  Vector3d box_min_, box_max_;//包含该边界最小轴对称立方体
  // Path and cost from this cluster to other clusters
  list<vector<Vector3d>> paths_;//与其他边界的连接路径
  list<double> costs_;//连接其他边界的代价
  //区域编号，采用vectoe3d存储，分别为（z，y，x）
  Vector3d Region_ID;
  //根据区域优先级的到的视点优先级
  int priority_in_region;
};

//定义Region结构体存储每一个区域信息
struct RegionNode{
    //当前区域编号
    Eigen::Vector3i region_id;
    //当前区域顺序
    int serial_number;
    //区域中心（用以求解区域之间的代价）
    Vector3d region_center;
    //视点处在该区域的边界集合
    list<Frontier> frontier_in_region;
    //连接其他边界的代价
    list<double> costs_region;
};

class FrontierFinder {
public:
  FrontierFinder(const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh);
  ~FrontierFinder();

  void searchFrontiers();
  void computeFrontiersToVisit();

  /*双层TSP相关函数*/
  //获取当前边界的Region_ID信息
  void getRegionInfo();
  void getRegionMax();
  void getCurrentID(const Vector3d& odom_, Eigen::Vector3i& cur_id_);
  void inicial_region_list(list<RegionNode>& region_list);
  void GetNewExternalCostMatrix(const Vector3d& cur_pos, const Vector3d& cur_v, Eigen::MatrixXd& ExternalTspCostMatrix);
  void UpdateExternalCost(const list<RegionNode>::iterator& it1, const list<RegionNode>::iterator& it2);
  double ComputeExternalCost(const Vector3d& ct_1, const Vector3d& ct_2, const Eigen::Vector3i& id_1, const Eigen::Vector3i& id_2);
  double ComputeFirstStarToOthers(const Vector3d& other_region, const Vector3d& cur_p,
         const Vector3d& cur_v, const Eigen::Vector3i& id_other, const Eigen::Vector3i& id_now);
  void UpdateExternalTSPList(Eigen::MatrixXd& ExternalTspCostMatrix);
  void GetPriorityRegionList(const list<RegionNode>& region_node_list_last_, vector<int>& priority_list);
  void getViewPriority();
  // double GetNewInternalCostMatrix(const Vector3d& p1, const Vector3d& p2, const double& y1, const double& y2,
  //                            const Vector3d& v1, const double& yd1, vector<Vector3d>& path,
  //                            const int& priority1, const int& priority2);

  void getFrontiers(vector<vector<Vector3d>>& clusters);
  void getDormantFrontiers(vector<vector<Vector3d>>& clusters);
  void getFrontierBoxes(vector<pair<Vector3d, Vector3d>>& boxes);
  // Get viewpoint with highest coverage for each frontier
  void getTopViewpointsInfo(const Vector3d& cur_pos, vector<Vector3d>& points, vector<double>& yaws,
                            vector<Vector3d>& averages);
  // Get several viewpoints for a subset of frontiers
  void getViewpointsInfo(const Vector3d& cur_pos, const vector<int>& ids, const int& view_num,
                         const double& max_decay, vector<vector<Vector3d>>& points,
                         vector<vector<double>>& yaws);
  void updateFrontierCostMatrix();
  void getFullCostMatrix(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
                         Eigen::MatrixXd& mat);
  void getPathForTour(const Vector3d& pos, const vector<int>& frontier_ids, vector<Vector3d>& path);

  void setNextFrontier(const int& id);
  bool isFrontierCovered();
  void wrapYaw(double& yaw);

  shared_ptr<PerceptionUtils> percep_utils_;

  //当前所在位置的区域ID：
  Eigen::Vector3i cur_id;
  //外部TSP参数
  double wd,wb,wh,wc;
  // //内部TSP参数
  // double vm,w_dir,wl,wy,wp;
  //是否为第一次分割（第一才需要分割标志位）
  bool need_splite = 1;
  //已经完成外部区域TSP优化标志位
  bool need_external_tsp = 1;
  //已经完成优先级排序标志位
  bool need_get_priority = 1;
  //外部TSP代价矩阵
  Eigen::MatrixXd ExternalTspCostMatrix_;
  //优先级查询数组
  vector<int> priority_list;

  //param
  list<RegionNode> region_node_list;
  list<RegionNode> region_node_list_last;

private:
  void splitLargeFrontiers(list<Frontier>& frontiers);

  bool splitHorizontally(const Frontier& frontier, list<Frontier>& splits);
  void mergeFrontiers(Frontier& ftr1, const Frontier& ftr2);
  bool isFrontierChanged(const Frontier& ft);
  bool haveOverlap(const Vector3d& min1, const Vector3d& max1, const Vector3d& min2,
                   const Vector3d& max2);
  void computeFrontierInfo(Frontier& frontier);
  void downsample(const vector<Vector3d>& cluster_in, vector<Vector3d>& cluster_out);
  void sampleViewpoints(Frontier& frontier);
  Vector3d GetYawRange(Frontier& Frontier);

  int countVisibleCells(const Vector3d& pos, const double& yaw, const vector<Vector3d>& cluster);
  bool isNearUnknown(const Vector3d& pos);
  vector<Eigen::Vector3i> sixNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> tenNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i& voxel);
  bool isNeighborUnknown(const Eigen::Vector3i& voxel);
  void expandFrontier(const Eigen::Vector3i& first /* , const int& depth, const int& parent_id */);

  // Wrapper of sdf map
  int toadr(const Eigen::Vector3i& idx);
  bool knownfree(const Eigen::Vector3i& idx);
  bool inmap(const Eigen::Vector3i& idx);

  // Deprecated
  Eigen::Vector3i searchClearVoxel(const Eigen::Vector3i& pt);
  bool isInBoxes(const vector<pair<Vector3d, Vector3d>>& boxes, const Eigen::Vector3i& idx);
  bool canBeMerged(const Frontier& ftr1, const Frontier& ftr2);
  void findViewpoints(const Vector3d& sample, const Vector3d& ftr_avg, vector<Viewpoint>& vps);


  // Data
  vector<char> frontier_flag_;//边界标志位
  list<Frontier> frontiers_, dormant_frontiers_, tmp_frontiers_;// 边界列表、无视点边界列表 、临时列表
  vector<int> removed_ids_;//记录需要移除的前沿id
  list<Frontier>::iterator first_new_ftr_;
  Frontier next_frontier_;

  // Params
  int cluster_min_;
  double cluster_size_xy_, cluster_size_z_;
  double candidate_rmax_, candidate_rmin_, candidate_dphi_, min_candidate_dist_,
      min_candidate_clearance_;

  //双层TSP相关
  //计算探索区域地图范围
  double regin_min_x,regin_max_x,regin_min_y,regin_max_y,regin_min_z,regin_max_z;
  //定义区域最大范围:
  int ID_max_x,ID_max_y,ID_max_z;
  //区域分割阈值：
  double splite_x,splite_y,splite_z;
  //外部tsp文件命名空间
  std::string external_tsp_dir;


  int down_sample_;
  double min_view_finish_fraction_, resolution_;
  int min_visib_num_, candidate_rnum_;

  // Utils
  shared_ptr<EDTEnvironment> edt_env_;
  unique_ptr<RayCaster> raycaster_;
};

}  // namespace fast_planner
#endif