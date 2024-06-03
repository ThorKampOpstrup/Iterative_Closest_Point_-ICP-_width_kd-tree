#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <kdtree.h>

int main(int argc, char** argv) {
  rclcpp::Clock system_clock;

  if(argc != 3) {
    std::cout << "usage: kd_tree_from_pc input.pcd threshold" << std::endl;
    return 0;
  }
  std::string input_pcd = argv[1];
  float threshold = std::stoi(argv[2]);
  //   std::cout << "hello" << std::endl;
  std::cout << "threshold: " << threshold << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>());

  if(pcl::io::loadPCDFile(input_pcd, *pc)) {
    std::cerr << "failed to load " << input_pcd << std::endl;
    return 0;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());

  pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
  voxelgrid.setLeafSize(0.001f, 0.001f, 0.001f);

  auto t1 = system_clock.now();
  pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZI>());
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("intensity", pcl::ComparisonOps::GE, threshold)));

  pcl::ConditionalRemoval<pcl::PointXYZI> condrem;

  std::cout << "T: PointCloud before filtering: " << pc->width * pc->height << " data points (" << pcl::getFieldsList(*pc) << ")." << std::endl;

  voxelgrid.setInputCloud(pc);
  voxelgrid.filter(*downsampled);
  *pc = *downsampled;
  auto t2 = system_clock.now();
  std::cout << "downsampling took : " << (t2 - t1).seconds() * 1000 << "[msec]" << std::endl;
  std::cout << "T: PointCloud after filtering: " << pc->width * pc->height << " data points (" << pcl::getFieldsList(*pc) << ")." << std::endl;

  condrem.setInputCloud(pc);
  condrem.setCondition(range_cond);
  condrem.setKeepOrganized(false);  // Set to true if you want to keep the cloud organized

  condrem.filter(*pc);

  std::cout << "T: PointCloud after intensity value threshold: " << pc->width * pc->height << " data points (" << pcl::getFieldsList(*pc) << ")." << std::endl;

  pcl::io::savePCDFileASCII("tmp_pc.pcd", *pc);

  // read tmp_target.pcd and tmp_source.pcd
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_xyz(new pcl::PointCloud<pcl::PointXYZ>());

  if(pcl::io::loadPCDFile("tmp_pc.pcd", *pc_xyz)) {
    std::cerr << "failed to load " << pc_xyz << std::endl;
    return 0;
  }

  // start build kd- tree
  kdtree* kdtree_ptr = new kdtree();

  t1 = system_clock.now();
  std::cout << "allocating memory for kdtree" << std::endl;
  if(!kdtree_allocate(kdtree_ptr, 3, pc_xyz->width * pc_xyz->height)) {
    std::cerr << "failed to allocate memory for kdtree" << std::endl;
    return 0;
  }
  t2 = system_clock.now();
  std::cout << "kdtree allocated" << std::endl;
  std::cout << "kd-tree allocation took : " << (t2 - t1).seconds() * 1000 << "[msec]" << std::endl;

  /*insert some points...*/
  t1 = system_clock.now();
  std::cout << "inserting points" << std::endl;
  kdtree_insert(*kdtree_ptr, pc_xyz);
  t2 = system_clock.now();
  std::cout << "points inserted" << std::endl;
  std::cout << "kd-tree insertion took : " << (t2 - t1).seconds() * 1000 << "[msec]" << std::endl;

  // int result_size = kd_tree_in_order_traversal(root, max_cols);
  std::cout << "kd-tree size: " << kdtree_ptr->number_of_nodes << std::endl;

  // kdtree_print_depth(*kdtree_ptr);
  float query_point[3] = {0, 0, 0};

  // 1.15818 y: 0.907671 z: 0.783819
  //! SEARCH KD-TREE
  std::cout << "searching kd-tree" << std::endl;
  auto t3 = system_clock.now();
  for(uint i = 0; i < pc_xyz->width * pc_xyz->height; i++) {
    t1 = system_clock.now();
    query_point[0] = pc_xyz->points[i].x;
    query_point[1] = pc_xyz->points[i].y;
    query_point[2] = pc_xyz->points[i].z;
    kdtree_node* closest = kdtree_NN(*kdtree_ptr, query_point);
    std::cout << "Closest point to query point: ";
    kdtree_print_node_info(*kdtree_ptr, closest);
    std::cout << "distance: " << kdtree_distance(*kdtree_ptr, query_point, closest->data) << std::endl;
    t2 = system_clock.now();
    std::cout << "kd-tree search took : " << (t2 - t1).seconds() * 1000 << "[msec]" << std::endl;
  }
  std::cout << "searching through all points took : " << (system_clock.now() - t3).seconds() * 1000 << "[msec]" << std::endl;

  kdtree_node* closest = kdtree_NN(*kdtree_ptr, query_point);

  std::cout << "Closest point to query point: ";
  kdtree_print_node_info(*kdtree_ptr, closest);
  std::cout << "distance: " << kdtree_distance(*kdtree_ptr, query_point, closest->data) << std::endl;

  std::cout << "searching thought all points" << std::endl;
  t1 = system_clock.now();
  float lowest_distance, distance;
  for(int i = 0; i < kdtree_ptr->number_of_nodes; i++) {
    kdtree_node* node = kdtree_ptr->root + i;
    distance = kdtree_distance(*kdtree_ptr, query_point, node->data);
    if(i == 0) {
      lowest_distance = distance;
    }
    if(distance < lowest_distance) {
      lowest_distance = distance;
    }
  }
  t2 = system_clock.now();
  std::cout << "kd-tree search through all points took : " << (t2 - t1).seconds() * 1000 << "[msec]" << std::endl;
  std::cout << "Lowest distance found to be: " << lowest_distance << std::endl;

  // free the allocated memory
  std::cout << "freeing memory" << std::endl;
  kdtree_free(kdtree_ptr);
  return 0;
}
