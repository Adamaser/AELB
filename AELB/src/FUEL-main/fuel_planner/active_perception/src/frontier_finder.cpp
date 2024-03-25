#include <active_perception/frontier_finder.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>
// #include <path_searching/astar2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plan_env/edt_environment.h>
#include <active_perception/perception_utils.h>
#include <active_perception/graph_node.h>

// use PCL region growing segmentation
// #include <pcl/point_types.h>
// #include <pcl/search/search.h>
// #include <pcl/search/kdtree.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Eigenvalues>

#include <thread>
#include <iostream>
#include <fstream>
#include <lkh_tsp_solver/lkh_interface.h>

namespace fast_planner {
  //构造函数，订阅话题、初始化参数
FrontierFinder::FrontierFinder(const EDTEnvironment::Ptr& edt, ros::NodeHandle& nh) {
  this->edt_env_ = edt;
  int voxel_num = edt->sdf_map_->getVoxelNum();
  frontier_flag_ = vector<char>(voxel_num, 0);
  fill(frontier_flag_.begin(), frontier_flag_.end(), 0);

  nh.param("frontier/cluster_min", cluster_min_, -1);
  nh.param("frontier/cluster_size_xy", cluster_size_xy_, -1.0);
  nh.param("frontier/cluster_size_z", cluster_size_z_, -1.0);
  nh.param("frontier/min_candidate_dist", min_candidate_dist_, -1.0);
  nh.param("frontier/min_candidate_clearance", min_candidate_clearance_, -1.0);
  nh.param("frontier/candidate_dphi", candidate_dphi_, -1.0);
  nh.param("frontier/candidate_rmax", candidate_rmax_, -1.0);
  nh.param("frontier/candidate_rmin", candidate_rmin_, -1.0);
  nh.param("frontier/candidate_rnum", candidate_rnum_, -1);
  nh.param("frontier/down_sample", down_sample_, -1);
  nh.param("frontier/min_visib_num", min_visib_num_, -1);
  nh.param("frontier/min_view_finish_fraction", min_view_finish_fraction_, -1.0);

  //以下为修改计算区域的参数配置导入
  nh.param("sdf_map/box_min_x", regin_min_x, -10.0);
  nh.param("sdf_map/box_max_x", regin_max_x, 10.0);
  nh.param("sdf_map/box_min_y", regin_min_y, -10.0);
  nh.param("sdf_map/box_max_y", regin_max_y, 10.0);
  nh.param("sdf_map/box_min_z", regin_min_z, 0.0);
  nh.param("sdf_map/box_max_z", regin_max_z, 2.0);

  //读取无人机分割阈值
  nh.param("splite/x", splite_x, 5.0);
  nh.param("splite/y", splite_y, 5.0);
  nh.param("splite/z", splite_z, 2.0);

  //读取外部tsp系数
  nh.param("external_tsp/wd", wd, 5.0);
  nh.param("external_tsp/wb", wb, 5.0);
  nh.param("external_tsp/wh", wh, 5.0);
  nh.param("external_tsp/wc", wc, 5.0);

  //内部TSP参数
  // nh.param("internal_tsp/vm", vm, 0.5);
  // nh.param("internal_tsp/w_dir", w_dir, 0.5);
  // nh.param("internal_tsp/wy", wy, 1);
  // nh.param("internal_tsp/wp", wp, 1);
  // nh.param("internal_tsp/wp", wp, 1);

  //输入流数据位置
  nh.param("exploration/tsp_dir", external_tsp_dir, string("null"));

  raycaster_.reset(new RayCaster);
  resolution_ = edt_env_->sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  edt_env_->sdf_map_->getRegion(origin, size);
  raycaster_->setParams(resolution_, origin);

  std::cout <<"初始化成功"<<endl;
  //获取边界最大ID
  getRegionMax();
  std::cout <<"计算区域id成功"<<endl;
  //区域分割初始化
  inicial_region_list(region_node_list);

  percep_utils_.reset(new PerceptionUtils(nh));

  // Initialize TSP par file
  ofstream par_file(external_tsp_dir + "/external.par");
  par_file << "PROBLEM_FILE = " << external_tsp_dir << "/external.tsp\n";
  par_file << "GAIN23 = NO\n";
  par_file << "OUTPUT_TOUR_FILE =" << external_tsp_dir << "/external.txt\n";
  par_file << "RUNS = 1\n";
}

FrontierFinder::~FrontierFinder() {
}

//边界搜索函数
void FrontierFinder::searchFrontiers() {
  ros::Time t1 = ros::Time::now();
  tmp_frontiers_.clear();//清空临时列表

  // Bounding box of updated region
  Vector3d update_min, update_max; //更新区域的对角顶点
  edt_env_->sdf_map_->getUpdatedBox(update_min, update_max, true);//更新对角顶点值

  // 删除更新地图中变化的边界
  auto resetFlag = [&](list<Frontier>::iterator& iter, list<Frontier>& frontiers) {
    Eigen::Vector3i idx;
    for (auto cell : iter->cells_) {//边界中每一个点
      edt_env_->sdf_map_->posToIndex(cell, idx);//将cell的位置转换为栅格坐标
      frontier_flag_[toadr(idx)] = 0;//将其标志为0
    }
    iter = frontiers.erase(iter);//从前沿群列表中删除该前沿
  };

  std::cout << "Before remove: " << frontiers_.size() << std::endl;

  removed_ids_.clear();//初始化删除列表为空
  int rmv_idx = 0;//初始化删除的索引为0
  for (auto iter = frontiers_.begin(); iter != frontiers_.end();) {//遍历边界列中的每一组边界
    if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) &&
        isFrontierChanged(*iter)) {//更新部分立方体与原有边界立方体有交集且边界发生变化
      resetFlag(iter, frontiers_);//删除当前受影响的前沿
      removed_ids_.push_back(rmv_idx);//将删除前沿的序号记录到待删除序号列表中
    } else {
      ++rmv_idx;
      ++iter;
    }
  }
  std::cout << "After remove: " << frontiers_.size() << std::endl;
  //dormant_frontiers_为当前不受更新影响的边界，即休眠状态边界
  for (auto iter = dormant_frontiers_.begin(); iter != dormant_frontiers_.end();) {
    if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) &&
        isFrontierChanged(*iter))
      resetFlag(iter, dormant_frontiers_);
    else
      ++iter;
  }

  // 在从更新的框中略微膨胀的框中搜索新的边界
  Vector3d search_min = update_min - Vector3d(1, 1, 0.5);
  Vector3d search_max = update_max + Vector3d(1, 1, 0.5);
  Vector3d box_min, box_max;
  edt_env_->sdf_map_->getBox(box_min, box_max);
  //确保三个维度上的搜索边界框不越界
  for (int k = 0; k < 3; ++k) {
    search_min[k] = max(search_min[k], box_min[k]);
    search_max[k] = min(search_max[k], box_max[k]);
  }
  Eigen::Vector3i min_id, max_id;
  edt_env_->sdf_map_->posToIndex(search_min, min_id);
  edt_env_->sdf_map_->posToIndex(search_max, max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        //扫描更新的区域以寻找边界的种子
        Eigen::Vector3i cur(x, y, z);
        if (frontier_flag_[toadr(cur)] == 0 && knownfree(cur) && isNeighborUnknown(cur)) {//非边界、非障碍物、至少有一个面上的邻居栅格为未知栅格
          // 拓展当前边界种子得到完整边界
          expandFrontier(cur);
        }
      }
  splitLargeFrontiers(tmp_frontiers_);//分割较大的边界

  ROS_WARN_THROTTLE(5.0, "Frontier t: %lf", (ros::Time::now() - t1).toSec());
}

//将传入的索引值标记为边界
void FrontierFinder::expandFrontier(
    const Eigen::Vector3i& first /* , const int& depth, const int& parent_id */) {
  // std::cout << "depth: " << depth << std::endl;
  auto t1 = ros::Time::now();

  // Data for clustering
  queue<Eigen::Vector3i> cell_queue;
  vector<Eigen::Vector3d> expanded;
  Vector3d pos;
  //通过索引获取三维位置
  edt_env_->sdf_map_->indexToPos(first, pos);
  //将三维位置信息加入到扩展列表
  expanded.push_back(pos);
  //索引加入到栅格列表
  cell_queue.push(first);
  //将其标记位边界
  frontier_flag_[toadr(first)] = 1;

  // Search frontier cluster based on region growing (distance clustering)
  while (!cell_queue.empty()) {
    auto cur = cell_queue.front();
    cell_queue.pop();
    auto nbrs = allNeighbors(cur);
    //遍历当前节点的每一个邻居
    for (auto nbr : nbrs) {
      // Qualified cell should be inside bounding box and frontier cell not clustered
      int adr = toadr(nbr);
      if (frontier_flag_[adr] == 1 || !edt_env_->sdf_map_->isInBox(nbr) ||//当其为边界、不在当前boundingbox、为障碍物或没有未知邻居节点
          !(knownfree(nbr) && isNeighborUnknown(nbr)))
        continue;
      //剩下的节点为非边界、在box、不为障碍物、有邻居节点
      edt_env_->sdf_map_->indexToPos(nbr, pos);
      if (pos[2] < 0.4) continue;  // Remove noise close to ground
      expanded.push_back(pos);
      cell_queue.push(nbr);
      frontier_flag_[adr] = 1;
    }
  }
  //当未知点达到了一定的数量记为边界
  if (expanded.size() > cluster_min_) {
    // Compute detailed info
    Frontier frontier;
    frontier.cells_ = expanded;
    computeFrontierInfo(frontier);
    //记录临时边界
    tmp_frontiers_.push_back(frontier);
  }
}

//分割较大的边界
void FrontierFinder::splitLargeFrontiers(list<Frontier>& frontiers) {
  //创建两个列表，splits 用于存储分割后的前沿，而 tmps 用于存储最终的前沿列表。
  list<Frontier> splits, tmps;
  //遍历每一个边界
  for (auto it = frontiers.begin(); it != frontiers.end(); ++it) {
    // Check if each frontier needs to be split horizontally
    if (splitHorizontally(*it, splits)) {//判断是否需要水平分割。如果需要分割，则将分割后的前沿存储在 splits 列表中，否则将原始前沿存储在 tmps 列表中。
    //如果前沿需要分割，则将分割后的前沿添加到 tmps 列表中，并清空 splits 列表。
      tmps.insert(tmps.end(), splits.begin(), splits.end());
      splits.clear();
    } else
      tmps.push_back(*it);
  }
  frontiers = tmps;
}

//水平方向上二维平面上进行分割
bool FrontierFinder::splitHorizontally(const Frontier& frontier, list<Frontier>& splits) {
  // Split a frontier into small piece if it is too large
  auto mean = frontier.average_.head<2>();
  bool need_split = false;
  for (auto cell : frontier.filtered_cells_) {
    if ((cell.head<2>() - mean).norm() > cluster_size_xy_) {
      need_split = true;
      break;
    }
  }
  if (!need_split) return false;

  // Compute principal component
  // Covariance matrix of cells
  Eigen::Matrix2d cov;
  cov.setZero();
  for (auto cell : frontier.filtered_cells_) {
    Eigen::Vector2d diff = cell.head<2>() - mean;
    cov += diff * diff.transpose();
  }
  cov /= double(frontier.filtered_cells_.size());

  // Find eigenvector corresponds to maximal eigenvector
  Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
  auto values = es.eigenvalues().real();
  auto vectors = es.eigenvectors().real();
  int max_idx;
  double max_eigenvalue = -1000000;
  for (int i = 0; i < values.rows(); ++i) {
    if (values[i] > max_eigenvalue) {
      max_idx = i;
      max_eigenvalue = values[i];
    }
  }
  Eigen::Vector2d first_pc = vectors.col(max_idx);
  std::cout << "max idx: " << max_idx << std::endl;
  std::cout << "mean: " << mean.transpose() << ", first pc: " << first_pc.transpose() << std::endl;

  // Split the frontier into two groups along the first PC
  Frontier ftr1, ftr2;
  for (auto cell : frontier.cells_) {
    if ((cell.head<2>() - mean).dot(first_pc) >= 0)
      ftr1.cells_.push_back(cell);
    else
      ftr2.cells_.push_back(cell);
  }
  computeFrontierInfo(ftr1);
  computeFrontierInfo(ftr2);

  // Recursive call to split frontier that is still too large
  list<Frontier> splits2;
  if (splitHorizontally(ftr1, splits2)) {
    splits.insert(splits.end(), splits2.begin(), splits2.end());
    splits2.clear();
  } else
    splits.push_back(ftr1);

  if (splitHorizontally(ftr2, splits2))
    splits.insert(splits.end(), splits2.begin(), splits2.end());
  else
    splits.push_back(ftr2);

  return true;
}

//判断是否在box内的函数
bool FrontierFinder::isInBoxes(
    const vector<pair<Vector3d, Vector3d>>& boxes, const Eigen::Vector3i& idx) {
  Vector3d pt;
  edt_env_->sdf_map_->indexToPos(idx, pt);
  for (auto box : boxes) {
    // Check if contained by a box
    bool inbox = true;
    for (int i = 0; i < 3; ++i) {
      inbox = inbox && pt[i] > box.first[i] && pt[i] < box.second[i];
      if (!inbox) break;
    }
    if (inbox) return true;
  }
  return false;
}

//更新边界代价矩阵
void FrontierFinder::updateFrontierCostMatrix() {
  //输出当前 frontiers_ 中每个元素的 cost 和 path 的大小
  std::cout << "cost mat size before remove: " << std::endl;
  for (auto ftr : frontiers_)
    std::cout << "(" << ftr.costs_.size() << "," << ftr.paths_.size() << "), ";
  std::cout << "" << std::endl;

  //如果如果有要移除的元素（removed_ids_ 不为空），则删除 frontiers_ 中对应位置的 path 和 cost：oved_ids_ 不为空），则删除 frontiers_ 中对应位置的 path 和 cost：
  std::cout << "cost mat size remove: " << std::endl;
  if (!removed_ids_.empty()) {
    // Delete path and cost for removed clusters
    //first_new_ftr_为更新起点位置
    for (auto it = frontiers_.begin(); it != first_new_ftr_; ++it) {
      auto cost_iter = it->costs_.begin();
      auto path_iter = it->paths_.begin();
      int iter_idx = 0;
      for (int i = 0; i < removed_ids_.size(); ++i) {
        // Step iterator to the item to be removed
        while (iter_idx < removed_ids_[i]) {
          ++cost_iter;
          ++path_iter;
          ++iter_idx;
        }
        cost_iter = it->costs_.erase(cost_iter);
        path_iter = it->paths_.erase(path_iter);
      }
      std::cout << "(" << it->costs_.size() << "," << it->paths_.size() << "), ";
    }
    removed_ids_.clear();
  }
  std::cout << "" << std::endl;

  //定义一个 lambda 函数 updateCost，用于计算两个 cluster 之间的路径和成本，并将它们添加到对应的 path 和 cost 中：
  auto updateCost = [](const list<Frontier>::iterator& it1, const list<Frontier>::iterator& it2) {
    std::cout << "(" << it1->id_ << "," << it2->id_ << "), ";
    // Search path from old cluster's top viewpoint to new cluster'
    Viewpoint& vui = it1->viewpoints_.front();
    Viewpoint& vuj = it2->viewpoints_.front();
    int& priorityi = it1->priority_in_region;
    int& priorityj = it2->priority_in_region;
    vector<Vector3d> path_ij;
    double cost_ij = ViewNode::ComputeInternalCost(
        vui.pos_, vuj.pos_, vui.yaw_, vuj.yaw_, Vector3d(0, 0, 0), 0, path_ij, priorityi, priorityj);
    // Insert item for both old and new clusters
    it1->costs_.push_back(cost_ij);
    it1->paths_.push_back(path_ij);
    reverse(path_ij.begin(), path_ij.end());
    it2->costs_.push_back(cost_ij);
    it2->paths_.push_back(path_ij);
  };

  //输出添加新 path 和 cost 后 frontiers_ 中每个元素的大小：
  std::cout << "cost mat add: " << std::endl;
  // Compute path and cost between old and new clusters
  for (auto it1 = frontiers_.begin(); it1 != first_new_ftr_; ++it1)
    for (auto it2 = first_new_ftr_; it2 != frontiers_.end(); ++it2)
      updateCost(it1, it2);

  //输出最终 frontiers_ 中每个元素的 cost 和 path 的大小：
  for (auto it1 = first_new_ftr_; it1 != frontiers_.end(); ++it1)
    for (auto it2 = it1; it2 != frontiers_.end(); ++it2) {
      if (it1 == it2) {
        std::cout << "(" << it1->id_ << "," << it2->id_ << "), ";
        it1->costs_.push_back(0);
        it1->paths_.push_back({});
      } else
        updateCost(it1, it2);
    }
  std::cout << "" << std::endl;
  std::cout << "cost mat size final: " << std::endl;
  for (auto ftr : frontiers_)
    std::cout << "(" << ftr.costs_.size() << "," << ftr.paths_.size() << "), ";
  std::cout << "" << std::endl;
}

//合并边界
void FrontierFinder::mergeFrontiers(Frontier& ftr1, const Frontier& ftr2) {
  // Merge ftr2 into ftr1
  ftr1.average_ =
      (ftr1.average_ * double(ftr1.cells_.size()) + ftr2.average_ * double(ftr2.cells_.size())) /
      (double(ftr1.cells_.size() + ftr2.cells_.size()));
  ftr1.cells_.insert(ftr1.cells_.end(), ftr2.cells_.begin(), ftr2.cells_.end());
  computeFrontierInfo(ftr1);
}

//判断边界融合是否会超过大小限制
bool FrontierFinder::canBeMerged(const Frontier& ftr1, const Frontier& ftr2) {
  Vector3d merged_avg =
      (ftr1.average_ * double(ftr1.cells_.size()) + ftr2.average_ * double(ftr2.cells_.size())) /
      (double(ftr1.cells_.size() + ftr2.cells_.size()));
  // Check if it can merge two frontier without exceeding size limit
  for (auto c1 : ftr1.cells_) {
    auto diff = c1 - merged_avg;
    if (diff.head<2>().norm() > cluster_size_xy_ || diff[2] > cluster_size_z_) return false;
  }
  for (auto c2 : ftr2.cells_) {
    auto diff = c2 - merged_avg;
    if (diff.head<2>().norm() > cluster_size_xy_ || diff[2] > cluster_size_z_) return false;
  }
  return true;
}

//判断两个box是否有交叉部分
bool FrontierFinder::haveOverlap(
    const Vector3d& min1, const Vector3d& max1, const Vector3d& min2, const Vector3d& max2) {
  // Check if two box have overlap part
  Vector3d bmin, bmax;
  for (int i = 0; i < 3; ++i) {
    bmin[i] = max(min1[i], min2[i]);
    bmax[i] = min(max1[i], max2[i]);
    if (bmin[i] > bmax[i] + 1e-3) return false;
  }
  return true;
}

//判断边界是否发生改变
bool FrontierFinder::isFrontierChanged(const Frontier& ft) {
  for (auto cell : ft.cells_) {
    Eigen::Vector3i idx;
    edt_env_->sdf_map_->posToIndex(cell, idx);
    if (!(knownfree(idx) && isNeighborUnknown(idx))) return true;
  }
  return false;
}

//计算边界的平均位置与最小的box
void FrontierFinder::computeFrontierInfo(Frontier& ftr) {
  // Compute average position and bounding box of cluster
  ftr.average_.setZero();
  ftr.box_max_ = ftr.cells_.front();
  ftr.box_min_ = ftr.cells_.front();
  for (auto cell : ftr.cells_) {
    ftr.average_ += cell;
    for (int i = 0; i < 3; ++i) {
      ftr.box_min_[i] = min(ftr.box_min_[i], cell[i]);
      ftr.box_max_[i] = max(ftr.box_max_[i], cell[i]);
    }
  }
  ftr.average_ /= double(ftr.cells_.size());

  // Compute downsampled cluster
  downsample(ftr.cells_, ftr.filtered_cells_);
}

//计算边界视点与根据覆盖函数排序视点集合
void FrontierFinder::computeFrontiersToVisit() {
  first_new_ftr_ = frontiers_.end();
  int new_num = 0;
  int new_dormant_num = 0;
  // Try find viewpoints for each cluster and categorize them according to viewpoint number
  for (auto& tmp_ftr : tmp_frontiers_) {
    // Search viewpoints around frontier
    sampleViewpoints(tmp_ftr);
    if (!tmp_ftr.viewpoints_.empty()) {
      ++new_num;
      list<Frontier>::iterator inserted = frontiers_.insert(frontiers_.end(), tmp_ftr);
      // Sort the viewpoints by coverage fraction, best view in front
      sort(
          inserted->viewpoints_.begin(), inserted->viewpoints_.end(),
          [](const Viewpoint& v1, const Viewpoint& v2) { return v1.visib_num_ > v2.visib_num_; });
      if (first_new_ftr_ == frontiers_.end()) first_new_ftr_ = inserted;
    } else {
      // Find no viewpoint, move cluster to dormant list
      dormant_frontiers_.push_back(tmp_ftr);
      ++new_dormant_num;
    }
  }
  // Reset indices of frontiers
  int idx = 0;
  for (auto& ft : frontiers_) {
    ft.id_ = idx++;
    std::cout << ft.id_ << ", ";
  }
  std::cout << "\nnew num: " << new_num << ", new dormant: " << new_dormant_num << std::endl;
  std::cout << "to visit: " << frontiers_.size() << ", dormant: " << dormant_frontiers_.size()
            << std::endl;
}

//获取最优视点
void FrontierFinder::getTopViewpointsInfo(
    const Vector3d& cur_pos, vector<Eigen::Vector3d>& points, vector<double>& yaws,
    vector<Eigen::Vector3d>& averages) {
  points.clear();
  yaws.clear();
  averages.clear();
  for (auto frontier : frontiers_) {
    bool no_view = true;
    for (auto view : frontier.viewpoints_) {
      // Retrieve the first viewpoint that is far enough and has highest coverage
      if ((view.pos_ - cur_pos).norm() < min_candidate_dist_) continue;
      points.push_back(view.pos_);
      yaws.push_back(view.yaw_);
      averages.push_back(frontier.average_);
      no_view = false;
      break;
    }
    if (no_view) {
      // All viewpoints are very close, just use the first one (with highest coverage).
      auto view = frontier.viewpoints_.front();
      points.push_back(view.pos_);
      yaws.push_back(view.yaw_);
      averages.push_back(frontier.average_);
    }
  }
}

//获取所有边界最优视点
void FrontierFinder::getViewpointsInfo(
    const Vector3d& cur_pos, const vector<int>& ids, const int& view_num, const double& max_decay,
    vector<vector<Eigen::Vector3d>>& points, vector<vector<double>>& yaws) {
  points.clear();
  yaws.clear();
  for (auto id : ids) {
    // Scan all frontiers to find one with the same id
    for (auto frontier : frontiers_) {
      if (frontier.id_ == id) {
        // Get several top viewpoints that are far enough
        vector<Eigen::Vector3d> pts;
        vector<double> ys;
        int visib_thresh = frontier.viewpoints_.front().visib_num_ * max_decay;
        for (auto view : frontier.viewpoints_) {
          if (pts.size() >= view_num || view.visib_num_ <= visib_thresh) break;
          if ((view.pos_ - cur_pos).norm() < min_candidate_dist_) continue;
          pts.push_back(view.pos_);
          ys.push_back(view.yaw_);
        }
        if (pts.empty()) {
          // All viewpoints are very close, ignore the distance limit
          for (auto view : frontier.viewpoints_) {
            if (pts.size() >= view_num || view.visib_num_ <= visib_thresh) break;
            pts.push_back(view.pos_);
            ys.push_back(view.yaw_);
          }
        }
        points.push_back(pts);
        yaws.push_back(ys);
      }
    }
  }
}

//获取边界的所有栅格
void FrontierFinder::getFrontiers(vector<vector<Eigen::Vector3d>>& clusters) {
  clusters.clear();
  // std::cout << "此时新增的边界数量为" << frontiers_.size() << std::endl; 
  for (auto frontier : frontiers_){
    clusters.push_back(frontier.cells_);
    // std::cout << "新增加的边界id为" << frontier.Region_ID << std::endl; 
    }
  // clusters.push_back(frontier.filtered_cells_);
}

//获取睡眠状态的栅格
void FrontierFinder::getDormantFrontiers(vector<vector<Vector3d>>& clusters) {
  clusters.clear();
  for (auto ft : dormant_frontiers_)
    clusters.push_back(ft.cells_);
}

//获取边界的box
void FrontierFinder::getFrontierBoxes(vector<pair<Eigen::Vector3d, Eigen::Vector3d>>& boxes) {
  boxes.clear();
  for (auto frontier : frontiers_) {
    Vector3d center = (frontier.box_max_ + frontier.box_min_) * 0.5;
    Vector3d scale = frontier.box_max_ - frontier.box_min_;
    boxes.push_back(make_pair(center, scale));
  }
}

/*双层TSP相关函数*/
//获取当前边界的Region_ID信息
void FrontierFinder::getRegionInfo(){
  Vector3d temp_position;
  for (auto it = frontiers_.begin(); it != frontiers_.end(); ++it){
    temp_position = it->viewpoints_[0].pos_;
    it->Region_ID[2] = min(int((temp_position[2]-regin_min_z)/splite_z) ,ID_max_z-1);
    it->Region_ID[1] = min(int((temp_position[1]-regin_min_y)/splite_y) ,ID_max_y-1);
    it->Region_ID[0] = min(int((temp_position[0]-regin_min_x)/splite_x) ,ID_max_x-1);
    std::cout <<"视点坐标为"<< temp_position[0]<<" , "<<temp_position[1]<<" , "<<temp_position[2]<<std::endl;
    std::cout <<"新视点区域id为"<< it->Region_ID[0]<<" , "<<it->Region_ID[1]<<" , "<<it->Region_ID[2]<<std::endl;
  }
}

//根据区域序号更新边界搜索优先级
void FrontierFinder::getViewPriority(){
  for( auto it = frontiers_.begin(); it != frontiers_.end(); ++it){
    it->priority_in_region = priority_list[(it->Region_ID(0))*ID_max_y*ID_max_z + (it->Region_ID(1))*ID_max_z + (it->Region_ID(2))];
    std::cout <<"新视点区域id为" << it->Region_ID[0]<<" , "<<it->Region_ID[1]<<" , "<<it->Region_ID[2]<<std::endl;
    std::cout <<"新视点优先级为" << it-> priority_in_region;
  }
}

//获取区域编号范围
void FrontierFinder::getRegionMax(){
  if(int(regin_max_x-regin_min_x)%int(splite_x)== 0){
    ID_max_x = (regin_max_x-regin_min_x)/splite_x;
  }
  else{
    ID_max_x = (regin_max_x-regin_min_x)/splite_x + 1;
    splite_x = (regin_max_x-regin_min_x)/ID_max_x;
  }
  if(int(regin_max_y-regin_min_y)%int(splite_y)== 0){
    ID_max_y = (regin_max_y-regin_min_y)/splite_y;
  }
  else{
    ID_max_y = (regin_max_y-regin_min_y)/splite_y + 1;
    splite_y = (regin_max_y-regin_min_y)/ID_max_y;
  }
  if(int(regin_max_z-regin_min_z)%int(splite_z)== 0){
    ID_max_z = (regin_max_z-regin_min_z)/splite_z;
  }
  else{
    ID_max_z = (regin_max_z-regin_min_z)/splite_z + 1;
    splite_z = (regin_max_z-regin_min_z)/ID_max_z;
  }
  std::cout <<"当前地图尺寸为"<<regin_max_x-regin_min_x<<" , "<<regin_max_y-regin_min_y<<" , "<<regin_max_z-regin_min_z<<endl;
  std::cout <<"当前分割单位为"<<splite_x<<" , "<<splite_y<<" , "<<splite_z<<endl;
  std::cout <<"当前区域ID最大值为"<<ID_max_x-1<<" , "<<ID_max_y-1<<" , "<<ID_max_z-1<<endl;
}

//获取当前id信息,id(x,y,z)
void FrontierFinder::getCurrentID(const Vector3d& odom_, Vector3i& cur_id_){
  cur_id_(2) = min(int((odom_[2]-regin_min_z)/splite_z) ,ID_max_z);
  cur_id_(1) = min(int((odom_[1]-regin_min_y)/splite_y) ,ID_max_y);
  cur_id_(0) = min(int((odom_[0]-regin_min_x)/splite_x) ,ID_max_x);
  // std::cout << "无人机的当前坐标位置为：" << cur_id_(0) << ','<< cur_id_(1) << ','<< cur_id_(2) << std::endl;
}

//初始化外部TSP序列
void FrontierFinder::inicial_region_list(list<RegionNode>& region_list){
    RegionNode region_temp_node;
    //规定顺序x,y,z为i,j,k
    for(int i = 0; i < ID_max_x; i++){
        for(int j = 0; j < ID_max_y; j++){
            for(int k = 0; k < ID_max_z; k++){
                region_temp_node.region_id << i,j,k;
                region_temp_node.region_center <<  (i+0.5)*splite_x + regin_min_x,(j+0.5)*splite_y + regin_min_y,(k+0.5)*splite_z;
                region_list.push_back(region_temp_node);
            }
        }
    }
    std::cout <<"区域队列元素个数共有："<<region_list.size()<<endl;
}

//根据当前位置更新TSP外部序列双向连接代价矩阵
void FrontierFinder::GetNewExternalCostMatrix(const Vector3d& cur_pos, const Vector3d& cur_v, Eigen::MatrixXd& ExternalTspCostMatrix){
  //输出需要计算TSP代价的序列长度
  if(need_splite == 1){
    std::cout << "当前需要优化的序列长度为：" <<  region_node_list.size() << endl;
    for (auto it1 = region_node_list.begin(); it1 != region_node_list.end(); it1++){
      for (auto it2 = it1 ; it2 != region_node_list.end(); it2++)
      {
        UpdateExternalCost(it1,it2);
      }
    }
    need_splite = 0;
    int dim =ID_max_x*ID_max_y*ID_max_z;
    ExternalTspCostMatrix.resize(dim+1, dim+1);  
    int i = 1, j = 1;
    for (auto ftr : region_node_list) {
      for (auto cs : ftr.costs_region)
        ExternalTspCostMatrix(i, j++) = cs;
      ++i;
      j = 1;
    }

    //考虑起点并构建开环
    ExternalTspCostMatrix.leftCols<1>().setZero();
    //当前的起点ID为
    std::cout << "当前的起点ID为：" << std::endl <<  cur_id << std::endl;
    std::cout << "当前的起点的位置为：" << std::endl <<  cur_pos << std::endl;
    std::cout << "外部代价函数为(未加起点)：" << std::endl <<  ExternalTspCostMatrix << std::endl;
    for (auto ftr : region_node_list) {
      ExternalTspCostMatrix(0, j++) = ComputeFirstStarToOthers(ftr.region_center, cur_pos, cur_v, ftr.region_id, cur_id);
    }
    std::cout << "外部代价函数为(已经加起点)：" << std::endl <<  ExternalTspCostMatrix << std::endl;
  }
    std::cout << "代价列表长度为" << region_node_list.begin()->costs_region.size() << std::endl;
}

//计算两点之间的代价：
void FrontierFinder::UpdateExternalCost(const list<RegionNode>::iterator& it1, const list<RegionNode>::iterator& it2){
  // std::cout << "(" << it1->region_id << "," << it2->region_id << "), ";
  Vector3d& ct1 = it1->region_center;
  Vector3d& ct2 = it2->region_center;
  Vector3i& id1 = it1->region_id;
  Vector3i& id2 = it2->region_id;
  double cost_12 = ComputeExternalCost(ct1,ct2,id1,id2);
  it1->costs_region.push_back(cost_12);
  if(it1 != it2){
    it2->costs_region.push_back(cost_12);
  }
}

//计算外部区域之间的代价计算函数
double FrontierFinder::ComputeExternalCost(const Vector3d& ct_1, const Vector3d& ct_2, const Vector3i& id_1, const Vector3i& id_2){
  //计算两者之间的曼哈顿距离
  double f_d = abs(ct_1(0)-ct_2(0)) + abs(ct_1(1)-ct_2(1));
  //边界代价函数
  double f_b = min(min(abs(id_2(0)-0),abs(id_2(0)-ID_max_x)),min(abs(id_2(1)-0),abs(id_2(1)-ID_max_y)));
  //高度分层代价函数
  double f_h = abs(id_1(2)-id_2(2));

  return wd*f_d + wb*f_b + wh*f_h ;
}

//计算起点到每个区域的代价
double FrontierFinder::ComputeFirstStarToOthers(const Vector3d& other_region, const Vector3d& cur_p, const Vector3d& cur_v, const Vector3i& id_other, const Vector3i& id_now){
  //打印当前区域中心位置
  std::cout << "当前遍历区域的中心位置为：" << std::endl <<  other_region << std::endl;
  //计算两者之间的曼哈顿距离
  double f_d = abs(cur_p(0)-other_region(0)) + abs(cur_p(1)-other_region(1));
  //边界启发函数
  double f_b = min(min(abs(id_other(0)-0),abs(id_other(0)-ID_max_x)),min(abs(id_other(1)-0),abs(id_other(1)-ID_max_y)));
  //高度分层代价函数
  double f_h = abs(id_now(2)-id_other(2));
  // double f_h = id_now(2)-id_other(2);
  //初始角度突变代价
  Vector3d dir = (cur_p-other_region).normalized();
  Vector3d vdir = cur_v.normalized();
  double f_c = acos(vdir.dot(dir));
  return wd*f_d + wb*f_b + wh*f_h + wc*f_c;
}

//采用TSP计算初始列表顺序
void FrontierFinder::UpdateExternalTSPList(Eigen::MatrixXd& ExternalTspCostMatrix){
  //记录当前运行时间
  auto t1 = ros::Time::now();
  const int dimension = ExternalTspCostMatrix.rows();
  //计算更新代价矩阵所花费的时间
  double mat_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  //创建了一个输出文件流prob_file，用于写入TSP问题文件
  ofstream prob_file(external_tsp_dir + "/external.tsp");
  
  std::string prob_spec = "NAME : external\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
      "\nEDGE_WEIGHT_TYPE : "
      "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";
  
  //将相关部分写入读取文件
  prob_file << prob_spec;

  //定义了一个缩放因子scale，用于缩放代价矩阵中的值
    const int scale = 100;
  if (false) {
    // Use symmetric TSP
    for (int i = 1; i < dimension; ++i) {
      for (int j = 0; j < i; ++j) {
        int int_cost = ExternalTspCostMatrix(i, j) * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }

  } else {
    // Use Asymmetric TSP
    for (int i = 0; i < dimension; ++i) {
      for (int j = 0; j < dimension; ++j) {
        int int_cost = ExternalTspCostMatrix(i, j) * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }
  }

  //关闭问题文件
  prob_file << "EOF";
  prob_file.close();

  //调用LKH TSP求解器解决问题
  solveTSPLKH((external_tsp_dir+ "/external.par").c_str());

  // Read optimal tour from the tour section of result file
  //创建一个输入文件流res_file，用于读取结果文件。
  ifstream res_file(external_tsp_dir + "/external.txt");
  string res;
  //在结果文件中查找并跳过到达“TOUR_SECTION”部分。
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0) break;
  }
  vector<int> indices;
  //从文件中读取优化结果
  while (getline(res_file, res)) {
      // Read indices of frontiers in optimal tour
      int id = stoi(res);
      if (id == 1)  // Ignore the current state
        continue;
      if (id == -1) break;
      indices.push_back(id - 2);  // Idx of solver-2 == Idx of frontier
    }
  //关闭优化文件
  res_file.close();

  //利用求解结果更新区域搜索顺序
  if(need_external_tsp == 1){
    for (int i : indices)
   {
    auto it = region_node_list.begin();
    std::advance(it, i);
    region_node_list_last.push_back(*it);
   }
   std::cout << "当前需要优化的队列长度为：" << region_node_list_last.size() << std::endl;
   need_external_tsp = 0;
  }
  
  std::cout<< "原区域搜索列表如下所示" << std::endl;
  for (const auto& node : region_node_list) {
        std::cout << "(" << node.region_id.x() << ", " << node.region_id.y() << ", " << node.region_id.z() << ")" << std::endl;
    }
  
  std::cout<< "更新后的区域搜索列表如下所示" << std::endl;
  for (const auto& node : region_node_list_last) {
        std::cout << "(" << node.region_id.x() << ", " << node.region_id.y() << ", " << node.region_id.z() << ")" << std::endl;
    }
}

//通过生成的优先序列生成每个区域的优先级
void FrontierFinder::GetPriorityRegionList(const list<RegionNode>& region_node_list_last_, vector<int>& priority_list_){
  if(need_get_priority==1){
    priority_list_.resize(ID_max_x*ID_max_y*ID_max_z);
    int priority_number = 1;
    for (const auto& node : region_node_list_last_){
      priority_list_[(node.region_id(0))*ID_max_y*ID_max_z + (node.region_id(1))*ID_max_z + node.region_id(2)] = priority_number;
      priority_number++;
      std::cout << "当前区域ID为" << node.region_id(0) <<','<< node.region_id(1) <<','<< node.region_id(2) <<std::endl;
      std::cout << "当前区域的搜索优先级为" << priority_list_[(node.region_id(0))*ID_max_y*ID_max_z + (node.region_id(1))*ID_max_z + node.region_id(2)];
    }
    need_get_priority = 0;
  }
}

//计算内部TSP代价函数
// double FrontierFinder::GetNewInternalCostMatrix(const Vector3d& p1, const Vector3d& p2, const double& y1, const double& y2,
//                              const Vector3d& v1, const double& yd1, vector<Vector3d>& path,
//                              const int& priority1, const int& priority2){
//   //求f_l
//   double pos_cost = ViewNode::searchPath(p1, p2, path)/ vm;
//   if (v1.norm() > 1e-3) {
//       Vector3d dir = (p2 - p1).normalized();
//       Vector3d vdir = v1.normalized();
//       double diff = acos(vdir.dot(dir));
//       pos_cost += w_dir * diff;
//     }
//   double diff = fabs(y2 - y1);
//   diff = min(diff, 2 * M_PI - diff);
//   double yaw_cost = diff / yd_;
//   double f_l = max(pos_cost, yaw_cost);
//   //求f_y?
//   //求f_p
//   double f_p = abs(priority2 - priority1);
//   return wl*f_l + wp*f_p;
// }

//获取覆盖路径
void FrontierFinder::getPathForTour(
    const Vector3d& pos, const vector<int>& frontier_ids, vector<Vector3d>& path) {
  // Make an frontier_indexer to access the frontier list easier
  vector<list<Frontier>::iterator> frontier_indexer;
  for (auto it = frontiers_.begin(); it != frontiers_.end(); ++it)
    frontier_indexer.push_back(it);

  // Compute the path from current pos to the first frontier
  vector<Vector3d> segment;
  ViewNode::searchPath(pos, frontier_indexer[frontier_ids[0]]->viewpoints_.front().pos_, segment);
  path.insert(path.end(), segment.begin(), segment.end());

  // Get paths of tour passing all clusters
  for (int i = 0; i < frontier_ids.size() - 1; ++i) {
    // Move to path to next cluster
    auto path_iter = frontier_indexer[frontier_ids[i]]->paths_.begin();
    int next_idx = frontier_ids[i + 1];
    for (int j = 0; j < next_idx; ++j)
      ++path_iter;
    path.insert(path.end(), path_iter->begin(), path_iter->end());
  }
}

//获取完整的代价矩阵
void FrontierFinder::getFullCostMatrix(
    const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
    Eigen::MatrixXd& mat) {
  if (false) {
    // Use symmetric TSP formulation
    int dim = frontiers_.size() + 2;
    mat.resize(dim, dim);  // current pose (0), sites, and virtual depot finally

    int i = 1, j = 1;
    for (auto ftr : frontiers_) {
      for (auto cs : ftr.costs_)
        mat(i, j++) = cs;
      ++i;
      j = 1;
    }

    // Costs from current pose to sites
    for (auto ftr : frontiers_) {
      Viewpoint vj = ftr.viewpoints_.front();
      vector<Vector3d> path;
      mat(0, j) = mat(j, 0) =
          ViewNode::computeCost(cur_pos, vj.pos_, cur_yaw[0], vj.yaw_, cur_vel, cur_yaw[1], path);
      ++j;
    }
    // Costs from depot to sites, the same large vaule
    for (j = 1; j < dim - 1; ++j) {
      mat(dim - 1, j) = mat(j, dim - 1) = 100;
    }
    // Zero cost to depot to ensure connection
    mat(0, dim - 1) = mat(dim - 1, 0) = -10000;

  } else {
    // Use Asymmetric TSP
    int dimen = frontiers_.size();
    mat.resize(dimen + 1, dimen + 1);
    // std::cout << "mat size: " << mat.rows() << ", " << mat.cols() << std::endl;
    // Fill block for clusters
    int i = 1, j = 1;
    for (auto ftr : frontiers_) {
      for (auto cs : ftr.costs_) {
        // std::cout << "(" << i << ", " << j << ")"
        // << ", ";
        mat(i, j++) = cs;
      }
      ++i;
      j = 1;
    }
    // std::cout << "" << std::endl;

    // Fill block from current state to clusters
    std::cout << "原文拓展前的代价矩阵为：" << std::endl << mat << std::endl;
    mat.leftCols<1>().setZero();
    for (auto ftr : frontiers_) {
      // std::cout << "(0, " << j << ")"
      // << ", ";
      Viewpoint vj = ftr.viewpoints_.front();
      vector<Vector3d> path;
      mat(0, j++) =
          ViewNode::computeCost(cur_pos, vj.pos_, cur_yaw[0], vj.yaw_, cur_vel, cur_yaw[1], path);
    }
    std::cout << "原文完整代价矩阵为：" << std::endl << mat << std::endl;
    // std::cout << "" << std::endl;
  }
}

//计算从不同的采样点不同的yaw角得到多个视点的信息
void FrontierFinder::findViewpoints(
    const Vector3d& sample, const Vector3d& ftr_avg, vector<Viewpoint>& vps) {
  if (!edt_env_->sdf_map_->isInBox(sample) ||
      edt_env_->sdf_map_->getInflateOccupancy(sample) == 1 || isNearUnknown(sample))
    return;

  double left_angle_, right_angle_, vertical_angle_, ray_length_;

  // Central yaw is determined by frontier's average position and sample
  auto dir = ftr_avg - sample;
  double hc = atan2(dir[1], dir[0]);

  vector<int> slice_gains;
  // Evaluate info gain of different slices
  for (double phi_h = -M_PI_2; phi_h <= M_PI_2 + 1e-3; phi_h += M_PI / 18) {
    // Compute gain of one slice
    int gain = 0;
    for (double phi_v = -vertical_angle_; phi_v <= vertical_angle_; phi_v += vertical_angle_ / 3) {
      // Find endpoint of a ray
      Vector3d end;
      end[0] = sample[0] + ray_length_ * cos(phi_v) * cos(hc + phi_h);
      end[1] = sample[1] + ray_length_ * cos(phi_v) * sin(hc + phi_h);
      end[2] = sample[2] + ray_length_ * sin(phi_v);

      // Do raycasting to check info gain
      Vector3i idx;
      raycaster_->input(sample, end);
      while (raycaster_->nextId(idx)) {
        // Hit obstacle, stop the ray
        if (edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 || !edt_env_->sdf_map_->isInBox(idx))
          break;
        // Count number of unknown cells
        if (edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) ++gain;
      }
    }
    slice_gains.push_back(gain);
  }

  // Sum up slices' gain to get different yaw's gain
  vector<pair<double, int>> yaw_gains;
  for (int i = 0; i < 6; ++i)  // [-90,-10]-> [10,90], delta_yaw = 20, 6 groups
  {
    double yaw = hc - M_PI_2 + M_PI / 9.0 * i + right_angle_;
    int gain = 0;
    for (int j = 2 * i; j < 2 * i + 9; ++j)  // 80 degree hFOV, 9 slices
      gain += slice_gains[j];
    yaw_gains.push_back(make_pair(yaw, gain));
  }

  // Get several yaws with highest gain
  vps.clear();
  sort(
      yaw_gains.begin(), yaw_gains.end(),
      [](const pair<double, int>& p1, const pair<double, int>& p2) {
        return p1.second > p2.second;
      });
  for (int i = 0; i < 3; ++i) {
    if (yaw_gains[i].second < min_visib_num_) break;
    Viewpoint vp = { sample, yaw_gains[i].first, yaw_gains[i].second };
    while (vp.yaw_ < -M_PI)
      vp.yaw_ += 2 * M_PI;
    while (vp.yaw_ > M_PI)
      vp.yaw_ -= 2 * M_PI;
    vps.push_back(vp);
  }
}

// 通过边界的平均位置、覆盖率采样得到视点
void FrontierFinder::sampleViewpoints(Frontier& frontier) {
  // Evaluate sample viewpoints on circles, find ones that cover most cells
  //离散化采样半径
  // Vector3d angel_get;
  double angle_mi,angle_ma;
  // angel_get = GetYawRange(frontier);
  angle_mi = -M_PI;
  angle_ma = M_PI;
  // std::cout << "************************************************************************当前角度范围为**********************************" <<angel_get[0] << angel_get[1]<< std::endl; 

  for (double rc = candidate_rmin_, dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_;
       rc <= candidate_rmax_ + 1e-3; rc += dr)
    //离散化采样角度
    for (double phi = angle_mi; phi < angle_ma; phi += candidate_dphi_) {
      //当前位置为中心，在周围取样
      const Vector3d sample_pos = frontier.average_ + rc * Vector3d(cos(phi), sin(phi), 0);
      // Qualified viewpoint is in bounding box and in safe region
      //保证视点在box内并且在安全区域内
      if (!edt_env_->sdf_map_->isInBox(sample_pos) ||
          edt_env_->sdf_map_->getInflateOccupancy(sample_pos) == 1 || isNearUnknown(sample_pos))
        continue;
      // Compute average yaw
      //引用当前边界栅格
      auto& cells = frontier.filtered_cells_;
      //计算采样点与边界第一个栅格之间的方向向量并且归一化得到参考方向
      Eigen::Vector3d ref_dir = (cells.front() - sample_pos).normalized();
      //初始化平均偏航角
      double avg_yaw = 0.0;
      //遍历单元格集合中的每个单元格，计算采样点与每个单元格之间的方向向量的夹角，并将夹角累加到 avg_yaw 中。
      for (int i = 1; i < cells.size(); ++i) {
        Eigen::Vector3d dir = (cells[i] - sample_pos).normalized();
        //返回 dir 和 ref_dir 之间的夹角。
        double yaw = acos(dir.dot(ref_dir));
        //角度为负，说明方向为逆时针的，需要取反
        if (ref_dir.cross(dir)[2] < 0) yaw = -yaw;
        avg_yaw += yaw;
      }
      //计算平均偏航角，取累加的夹角平均值，并加上参考方向的偏航角，最终调用 atan2 进行调整。
      avg_yaw = avg_yaw / cells.size() + atan2(ref_dir[1], ref_dir[0]);
      //对平均偏航角进行调整，可能是为了确保在合适的范围内
      wrapYaw(avg_yaw);
      // Compute the fraction of covered and visible cells
      //计算覆盖点信息
      int visib_num = countVisibleCells(sample_pos, avg_yaw, cells);
      if (visib_num > min_visib_num_) {
        Viewpoint vp = { sample_pos, avg_yaw, visib_num };
        frontier.viewpoints_.push_back(vp);
        // int gain = findMaxGainYaw(sample_pos, frontier, sample_yaw);
      }
      // }
    }
}

//自补充视点采样函数函数
Vector3d FrontierFinder::GetYawRange(Frontier& frontier){
  std::cout << "计算采样角度范围" << std::endl; 
  //Distinguish the middle point for Frontier
  Vector3d middle_temp,angle_range,vertex_temp;
  //Pseudo-2-D planar rectangle
  vector<Vector3d> vertex;
  //边框中点 
  vector<Vector3d> mid_bridge;
  //Temporary marker
  double temp_z;
  //求解边界框四个点
  temp_z = (frontier.box_max_[2]+ frontier.box_min_[2])/2;
  vertex_temp << frontier.box_max_[0],frontier.box_max_[1],temp_z;
  vertex.push_back(vertex_temp);
  vertex_temp << frontier.box_min_[0],frontier.box_max_[1],temp_z;
  vertex.push_back(vertex_temp);
  vertex_temp << frontier.box_min_[0],frontier.box_min_[1],temp_z;
  vertex.push_back(vertex_temp);
  vertex_temp << frontier.box_max_[0],frontier.box_min_[1],temp_z;
  vertex.push_back(vertex_temp);
  //求边框中点位置
  for(int index = 0; index < 3; index ++){
    middle_temp = (vertex[index]+vertex[index+1])/2;
    if (!isNearUnknown(middle_temp))
      continue;
    mid_bridge.push_back(middle_temp);
  }
  //计算采样角度
  int num_middle = mid_bridge.size();
  double angle_min = -M_PI, angle_max = M_PI;
  //定义临时方向
  Vector3d dir_temp;
  //临时采样角度
  double angle_temp,angle_temp1;
  //大于1个时进行排序
  std::vector<double> sort_angle;
  if(num_middle == 0){
    angle_range << angle_min,angle_max,0;
    return angle_range;
  }else if (num_middle == 1)
  {
    dir_temp = mid_bridge[0]-frontier.average_;
    angle_temp = atan2(dir_temp[1], dir_temp[0]);
    // wrapYaw(angle_temp);
    angle_range << angle_temp - M_PI/6, angle_temp + M_PI/6, 0;
    return angle_range;
  }else if (num_middle > 1)
  {
    for (int index = 0; index < num_middle; index++){
    dir_temp = mid_bridge[index]-frontier.average_;
    angle_temp = atan2(dir_temp[1], dir_temp[0]);
    // wrapYaw(angle_temp);
    sort_angle.push_back(angle_temp);
    }
    std::sort(sort_angle.begin(), sort_angle.end());
    angle_range << sort_angle.front(), sort_angle.back(), 0;
    return angle_range;
  }

  //公式计算viewpoint（calculate_viewpoint）
  //建立视点列表
}

//计算边界的覆盖信息
bool FrontierFinder::isFrontierCovered() {
  Vector3d update_min, update_max;
  edt_env_->sdf_map_->getUpdatedBox(update_min, update_max);

  auto checkChanges = [&](const list<Frontier>& frontiers) {
    for (auto ftr : frontiers) {
      if (!haveOverlap(ftr.box_min_, ftr.box_max_, update_min, update_max)) continue;
      const int change_thresh = min_view_finish_fraction_ * ftr.cells_.size();
      int change_num = 0;
      for (auto cell : ftr.cells_) {
        Eigen::Vector3i idx;
        edt_env_->sdf_map_->posToIndex(cell, idx);
        if (!(knownfree(idx) && isNeighborUnknown(idx)) && ++change_num >= change_thresh)
          return true;
      }
    }
    return false;
  };

  if (checkChanges(frontiers_) || checkChanges(dormant_frontiers_)) return true;

  return false;
}

//判断是否为最近的未知区域
bool FrontierFinder::isNearUnknown(const Eigen::Vector3d& pos) {
  const int vox_num = floor(min_candidate_clearance_ / resolution_);
  for (int x = -vox_num; x <= vox_num; ++x)
    for (int y = -vox_num; y <= vox_num; ++y)
      for (int z = -1; z <= 1; ++z) {
        Eigen::Vector3d vox;
        vox << pos[0] + x * resolution_, pos[1] + y * resolution_, pos[2] + z * resolution_;
        if (edt_env_->sdf_map_->getOccupancy(vox) == SDFMap::UNKNOWN) return true;
      }
  return false;
}

//计算可见栅格
int FrontierFinder::countVisibleCells(
    const Eigen::Vector3d& pos, const double& yaw, const vector<Eigen::Vector3d>& cluster) {
  percep_utils_->setPose(pos, yaw);
  int visib_num = 0;
  Eigen::Vector3i idx;
  for (auto cell : cluster) {
    // Check if frontier cell is inside FOV
    if (!percep_utils_->insideFOV(cell)) continue;

    // Check if frontier cell is visible (not occulded by obstacles)
    raycaster_->input(cell, pos);
    bool visib = true;
    while (raycaster_->nextId(idx)) {
      if (edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 ||
          edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
        visib = false;
        break;
      }
    }
    if (visib) visib_num += 1;
  }
  return visib_num;
}

//降采样函数
void FrontierFinder::downsample(
    const vector<Eigen::Vector3d>& cluster_in, vector<Eigen::Vector3d>& cluster_out) {
  // downsamping cluster
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudf(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto cell : cluster_in)
    cloud->points.emplace_back(cell[0], cell[1], cell[2]);

  const double leaf_size = edt_env_->sdf_map_->getResolution() * down_sample_;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor.filter(*cloudf);

  cluster_out.clear();
  for (auto pt : cloudf->points)
    cluster_out.emplace_back(pt.x, pt.y, pt.z);
}

//处理yaw角
void FrontierFinder::wrapYaw(double& yaw) {
  while (yaw < -M_PI)
    yaw += 2 * M_PI;
  while (yaw > M_PI)
    yaw -= 2 * M_PI;
}

//获取准确栅格
Eigen::Vector3i FrontierFinder::searchClearVoxel(const Eigen::Vector3i& pt) {
  queue<Eigen::Vector3i> init_que;
  vector<Eigen::Vector3i> nbrs;
  Eigen::Vector3i cur, start_idx;
  init_que.push(pt);
  // visited_flag_[toadr(pt)] = 1;

  while (!init_que.empty()) {
    cur = init_que.front();
    init_que.pop();
    if (knownfree(cur)) {
      start_idx = cur;
      break;
    }

    nbrs = sixNeighbors(cur);
    for (auto nbr : nbrs) {
      int adr = toadr(nbr);
      // if (visited_flag_[adr] == 0)
      // {
      //   init_que.push(nbr);
      //   visited_flag_[adr] = 1;
      // }
    }
  }
  return start_idx;
}

//对角线上的栅格
inline vector<Eigen::Vector3i> FrontierFinder::sixNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(6);
  Eigen::Vector3i tmp;

  tmp = voxel - Eigen::Vector3i(1, 0, 0);
  neighbors[0] = tmp;
  tmp = voxel + Eigen::Vector3i(1, 0, 0);
  neighbors[1] = tmp;
  tmp = voxel - Eigen::Vector3i(0, 1, 0);
  neighbors[2] = tmp;
  tmp = voxel + Eigen::Vector3i(0, 1, 0);
  neighbors[3] = tmp;
  tmp = voxel - Eigen::Vector3i(0, 0, 1);
  neighbors[4] = tmp;
  tmp = voxel + Eigen::Vector3i(0, 0, 1);
  neighbors[5] = tmp;

  return neighbors;
}

//获取当前栅格的邻居栅格
inline vector<Eigen::Vector3i> FrontierFinder::tenNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(10);
  Eigen::Vector3i tmp;
  int count = 0;

  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      if (x == 0 && y == 0) continue;
      tmp = voxel + Eigen::Vector3i(x, y, 0);
      neighbors[count++] = tmp;
    }
  }
  neighbors[count++] = tmp - Eigen::Vector3i(0, 0, 1);
  neighbors[count++] = tmp + Eigen::Vector3i(0, 0, 1);
  return neighbors;
}

//三维下的所有邻居栅格
inline vector<Eigen::Vector3i> FrontierFinder::allNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(26);
  Eigen::Vector3i tmp;
  int count = 0;
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y)
      for (int z = -1; z <= 1; ++z) {
        if (x == 0 && y == 0 && z == 0) continue;
        tmp = voxel + Eigen::Vector3i(x, y, z);
        neighbors[count++] = tmp;
      }
  return neighbors;
}

//判定是否为未知邻居
inline bool FrontierFinder::isNeighborUnknown(const Eigen::Vector3i& voxel) {
  // At least one neighbor is unknown
  auto nbrs = sixNeighbors(voxel);
  for (auto nbr : nbrs) {
    if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN) return true;
  }
  return false;
}

//取到准确未知
inline int FrontierFinder::toadr(const Eigen::Vector3i& idx) {
  return edt_env_->sdf_map_->toAddress(idx);
}

//判定是否为空
inline bool FrontierFinder::knownfree(const Eigen::Vector3i& idx) {
  return edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::FREE;
}

//判定是否在地图中
inline bool FrontierFinder::inmap(const Eigen::Vector3i& idx) {
  return edt_env_->sdf_map_->isInMap(idx);
}

}  // namespace fast_planner