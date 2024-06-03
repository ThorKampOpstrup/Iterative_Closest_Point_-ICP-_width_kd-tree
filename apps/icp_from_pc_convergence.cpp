#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "icp.h"

#include <Eigen/Eigen>

#define USE_LINEAR_SEARCH 1

#define ITT 50
#define TOL 0.00001

// translation(depth, sidewayes, up/down)
Eigen::Translation3d translation(0.4, 0.3, 0.4);

// angle-axis rotation

int main(int argc, char const *argv[]) {
  Eigen::Vector3d axis(1, 1, 1);
  // make axis unit vector
  axis.normalize();
  float angle = M_PI / 8;  // M_PI/32;
  Eigen::AngleAxisd rotation(angle, axis);

  rclcpp::Clock system_clock;

  if(argc != 3) {
    std::cout << "usage: kd_tree_from_pc input.pcd threshold" << std::endl;
    return 0;
  }
  std::string input_pcd = argv[1];
  float threshold = std::stoi(argv[2]);

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

  voxelgrid.setInputCloud(pc);
  voxelgrid.filter(*downsampled);
  *pc = *downsampled;
  auto t2 = system_clock.now();
  condrem.setInputCloud(pc);
  condrem.setCondition(range_cond);
  condrem.setKeepOrganized(false);  // Set to true if you want to keep the cloud organized

  condrem.filter(*pc);

  std::cout << pc->width * pc->height << std::endl;
  pcl::io::savePCDFileASCII("tmp_pc.pcd", *pc);

  // read tmp_target.pcd and tmp_source.pcd
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_xyz(new pcl::PointCloud<pcl::PointXYZ>());

  if(pcl::io::loadPCDFile("tmp_pc.pcd", *pc_xyz)) {
    std::cerr << "failed to load " << pc_xyz << std::endl;
    return 0;
  }

  // Generate kdtree
  t1 = system_clock.now();
  kdtree *tree_ptr = new kdtree();
  if(!kdtree_allocate(tree_ptr, 3, pc_xyz->width * pc_xyz->height)) {
    std::cerr << "failed to allocate kdtree" << std::endl;
    return 0;
  }

  kdtree_insert(*tree_ptr, pc_xyz);

  t2 = system_clock.now();
  // START ICP TEST
  Eigen::MatrixXd dst = pcl_to_eigen(pc_xyz);

  Eigen::Matrix4d transform = (translation * rotation).matrix();

  Eigen::Matrix3d R_transform = transform.block<3, 3>(0, 0);
  Eigen::Vector3d t_transform = transform.block<3, 1>(0, 3);

  Eigen::Matrix3d R_transform_inv = R_transform.transpose();
  Eigen::Vector3d t_transform_inv = -R_transform_inv * t_transform;

  Eigen::Matrix4d transform_inv_in = Eigen::Matrix4d::Identity();
  transform_inv_in.block<3, 3>(0, 0) = R_transform_inv;
  transform_inv_in.block<3, 1>(0, 3) = t_transform_inv;

  pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*pc_xyz, *rotated_cloud_xyz, transform);
  Eigen::MatrixXd src = pcl_to_eigen(rotated_cloud_xyz);

  Eigen::Matrix3d R;
  Eigen::Matrix4d T;

  ICP_OUT icp_result;

  t1 = system_clock.now();
  icp_result = icp_convergence(src, dst, ITT, TOL, transform_inv_in);
  t2 = system_clock.now();

  std::cout << (t2 - t1).seconds() * 1000 << std::endl;

  // // Visualization

  // pcl::visualization::PCLVisualizer vis("vis");
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_handler(pc_xyz, 255.0, 0.0, 0.0);
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_handler(rotated_cloud_xyz, 0.0, 255.0, 0.0);
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligned_handler(aligned_viz, 0.0, 0.0, 255.0);
  // vis.addPointCloud(pc_xyz, target_handler, "target");
  // vis.addPointCloud(rotated_cloud_xyz, source_handler, "source");
  // vis.addPointCloud(aligned_viz, aligned_handler, "aligned");
  // // add text to the viewer with colors
  // vis.addText("target", 10, 70, 12, 1.0, 0.0, 0.0, "target_text");
  // vis.addText("source", 10, 50, 12, 0.0, 1.0, 0.0, "source_text");
  // vis.addText("aligned", 10, 30, 12, 0.0, 0.0, 1.0, "aligned_text");
  // vis.setPosition(0, 0);
  // vis.spin();

  return 0;
}
