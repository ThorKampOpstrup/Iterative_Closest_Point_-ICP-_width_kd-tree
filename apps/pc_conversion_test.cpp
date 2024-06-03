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

#define ITT 100
#define TOL 0.000001

Eigen::Translation3d translation(0.1, 0.2, 0.3);

// angle-axis rotation
Eigen::Vector3d axis(1, 1, 1);
float angle = M_PI / 16;  // M_PI/32;
Eigen::AngleAxisd rotation(angle, axis);

double dist_to_center(pcl::PointXYZ p) {
  return std::sqrt((p.x * p.x) + (p.y * p.y) + (p.z * p.z));
}

double dist_to_center(Eigen::MatrixXd p) {
  return std::sqrt((p(0, 0) * p(0, 0)) + (p(1, 0) * p(1, 0)) + (p(2, 0) * p(2, 0)));
}

void print_dist_to_center(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) {
  double acc_dist = 0, dist = 0;
  for(int i = 0; i < pc->size(); i++) {
    dist = dist_to_center(pc->points[i]);
    acc_dist += dist;
    // print point coordinates
    //  std::cout << "pc point i: " << i << " x: " << pc->points[i].x << " y: " << pc->points[i].y << " z: " << pc->points[i].z << std::endl;
    std::cout << "pc point i: " << i << " dist to center: " << dist << std::endl;
  }
  std::cout << "acc_dist: " << acc_dist << std::endl;
  std::cout << std::endl;
}

void print_dist_to_center(Eigen::MatrixXd pc) {
  double acc_dist = 0, dist = 0;
  for(int i = 0; i < pc.rows(); i++) {
    dist = dist_to_center(pc.row(i));
    acc_dist += dist;
    // std::cout << "Eigen point i: " << i << " x: " << pc(i, 0) << " y: " << pc(i, 1) << " z: " << pc(i, 2) << std::endl;
    std::cout << "Eigen point i: " << i << " dist to center: " << dist << std::endl;
  }
  std::cout << "acc_dist: " << acc_dist << std::endl;
  std::cout << std::endl;
}

int main(int argc, char const *argv[]) {
  // define point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_org(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::PointXYZ p;
  p.x = 0;
  p.y = 0;
  p.z = 0;

  pc_org->push_back(p);

  p.x = 1;
  p.y = 0;
  p.z = 0;
  pc_org->push_back(p);

  p.x = 2;
  p.y = 2;
  p.z = 2;
  pc_org->push_back(p);

  p.x = 1;
  p.y = 1;
  p.z = 2;
  pc_org->push_back(p);

  p.x = -1;
  p.y = -2;
  p.z = -2;
  pc_org->push_back(p);

  p.x = 10;
  p.y = 0;
  p.z = 0;
  pc_org->push_back(p);

  p.x = 100;
  p.y = 0;
  p.z = 0;
  pc_org->push_back(p);

  Eigen::MatrixXd org = pcl_to_eigen(pc_org);

  //   rotation = rotation / rotation;
  // angle axis rotation to unit vector
  Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();
  Eigen::AngleAxisd rotation(rotation_matrix);
  Eigen::Matrix4d transform = (translation * rotation).matrix();
  //   Eigen::MatrixXd rotated = org * transform;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_rotated(new pcl::PointCloud<pcl::PointXYZ>());
  //   pc_rotated = eigen_to_pcl(rotated);
  pcl::transformPointCloud(*pc_org, *pc_rotated, transform);
  Eigen::MatrixXd rotated = pcl_to_eigen(pc_rotated);

  std::cout << "transform:\n\r " << transform << std::endl;
  std::cout << std::endl;

  std::cout << "Original Point Cloud: " << std::endl;
  print_dist_to_center(pc_org);
  std::cout << "Rotated Point Cloud: " << std::endl;
  print_dist_to_center(pc_rotated);

  std::cout << "Original Eigen Matrix: " << std::endl;
  print_dist_to_center(org);
  std::cout << "Rotated Eigen Matrix: " << std::endl;
  print_dist_to_center(rotated);

  ICP_OUT icp_result = icp(org, rotated, ITT, TOL);
  std::cout << "icp_result.trans\n\r" << icp_result.trans << std::endl;

  std::cout << "Original Eigen Matrix: After" << std::endl;
  print_dist_to_center(org);
  std::cout << "Rotated Eigen Matrix: After" << std::endl;
  print_dist_to_center(rotated);

  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_viz = eigen_to_pcl(rotated);
  Eigen::Matrix4d inv_trans = icp_result.trans.inverse();
  pcl::transformPointCloud(*aligned_viz, *aligned_viz, inv_trans);

  pcl::visualization::PCLVisualizer vis("vis");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_handler(pc_org, 255.0, 0.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_handler(pc_rotated, 0.0, 255.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligned_handler(aligned_viz, 0.0, 100, 255.0);
  vis.addPointCloud(pc_org, target_handler, "target");
  vis.addPointCloud(pc_rotated, source_handler, "source");
  vis.addPointCloud(aligned_viz, aligned_handler, "aligned");
  // add text to the viewer with colors
  vis.addText("target", 10, 10, 12, 1.0, 0.0, 0.0, "target_text");
  vis.addText("source", 10, 30, 12, 0.0, 1.0, 0.0, "source_text");
  vis.addText("aligned", 10, 50, 12, 0.0, 0.0, 1.0, "aligned_text");
  vis.setPosition(0, 0);
  vis.spin();

  return 0;
}