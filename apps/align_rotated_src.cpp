#include <iostream>
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>             //!added
#include <pcl/filters/conditional_removal.h>  //!added
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>

#include <std_msgs/msg/string.hpp>

// #define THRESHOLD 60

//translation(depth, sidewayes, up/down)
Eigen::Translation3f translation(0.1, 0.1, 0.2);

//angle-axis rotation
Eigen::Vector3f axis(1, 1, 1);
float angle = M_PI/16;//M_PI/32;
Eigen::AngleAxisf rotation(angle, axis);


// align point clouds and measure processing time
pcl::PointCloud<pcl::PointXYZ>::Ptr align(boost::shared_ptr<pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>> registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud) {
  registration->setInputTarget(target_cloud);
  registration->setInputSource(source_cloud);
  registration->setMaximumIterations(1000);
  registration->setMaxCorrespondenceDistance(0.30);
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());

  rclcpp::Clock system_clock;

  /// auto t1 = ros::WallTime::now();
  auto t1 = system_clock.now();
  registration->align(*aligned);
  auto t2 = system_clock.now();
  std::cout << "single : " << (t2 - t1).seconds() * 1000 << "[msec]" << std::endl;
  std::cout << "fitness: " << registration->getFitnessScore() << std::endl;
  
  
  //calculate the translational error
  Eigen::Vector3f calc_trans = registration->getFinalTransformation().block<3, 1>(0, 3);
  Eigen::Vector3f translation_error = calc_trans - translation.translation();
  std::cout << "translation_error: " << translation_error.norm() << std::endl;

  //caluculate the rotational error
  Eigen::Matrix3f calc_rot = registration->getFinalTransformation().block<3, 3>(0, 0);
  Eigen::Matrix3f rotation_error = (rotation).matrix() - calc_rot;
  std::cout << "rotation_error: " << rotation_error.norm() << std::endl;

  std::cout << std::endl;
  return aligned;
}

int main(int argc, char** argv) {
  if(argc != 3) {
    std::cout << "usage: align_rotated_src point_cloud threshold" << std::endl;
    return 0;
  }

  std::string source_pcd = argv[1];
  float threshold = std::stof(argv[2]);
  std::cout << "threshold: " << threshold << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZI>());

  if(pcl::io::loadPCDFile(source_pcd, *source_cloud)) {
    std::cerr << "failed to load " << source_pcd << std::endl;
    return 0;
  }

  // downsampling
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());

  pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
  voxelgrid.setLeafSize(0.02f, 0.02f, 0.02f);

  pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZI>());
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("intensity", pcl::ComparisonOps::GE, threshold)));

  pcl::ConditionalRemoval<pcl::PointXYZI> condrem;

  std::cout << "S: PointCloud before filtering: " << source_cloud->width * source_cloud->height << " data points (" << pcl::getFieldsList(*source_cloud) << ")." << std::endl;

  voxelgrid.setInputCloud(source_cloud);
  voxelgrid.filter(*downsampled);
  source_cloud = downsampled;

  std::cout << "S: PointCloud after filtering: " << source_cloud->width * source_cloud->height << " data points (" << pcl::getFieldsList(*source_cloud) << ")." << std::endl;

  condrem.setInputCloud(source_cloud);
  condrem.setCondition(range_cond);
  condrem.setKeepOrganized(false);  // Set to true if you want to keep the cloud organized

  condrem.filter(*source_cloud);

  std::cout << "S: PointCloud after intensity value threshold: " << source_cloud->width * source_cloud->height << " data points (" << pcl::getFieldsList(*source_cloud) << ")." << std::endl;

  // save tmp_target.pcd and tmp_source.pcd
  pcl::io::savePCDFileASCII("tmp_source.pcd", *source_cloud);

  // read tmp_target.pcd and tmp_source.pcd
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());

  if(pcl::io::loadPCDFile("tmp_source.pcd", *target_cloud_xyz)) {
    std::cerr << "failed to load " << target_cloud_xyz << std::endl;
    return 0;
  }
  // rotate target_cloud_xyz
  Eigen::Matrix4f transform = (translation * rotation).matrix();
  pcl::transformPointCloud(*target_cloud_xyz, *target_cloud_xyz, transform);

  if(pcl::io::loadPCDFile("tmp_source.pcd", *source_cloud_xyz)) {
    std::cerr << "failed to load " << source_cloud_xyz << std::endl;
    return 0;
  }

  // benchmark
  std::cout << "--- pcl::ICP ---" << std::endl;
  boost::shared_ptr<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>> icp(new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned = align(icp, target_cloud_xyz, source_cloud_xyz);

  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_viz = aligned;

  std::cout << "--- pcl::GICP ---" << std::endl;
  boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>> gicp(new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
  aligned = align(gicp, target_cloud_xyz, source_cloud_xyz);

  // TODO:The problem of uninitialized member variables
  std::cout << "--- pclomp::GICP ---" << std::endl;
  boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>> gicp_omp(new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
  aligned = align(gicp_omp, target_cloud_xyz, source_cloud_xyz);

  std::cout << "--- pcl::NDT ---" << std::endl;
  boost::shared_ptr<pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>> ndt(new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
  ndt->setResolution(1.0);
  aligned = align(ndt, target_cloud_xyz, source_cloud_xyz);

  std::vector<int> num_threads = {1, omp_get_max_threads()};
  std::vector<std::pair<std::string, pclomp::NeighborSearchMethod>> search_methods = {{"KDTREE", pclomp::KDTREE}, {"DIRECT7", pclomp::DIRECT7}, {"DIRECT1", pclomp::DIRECT1}};

  pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
  ndt_omp->setResolution(1.0);

  for(int n : num_threads) {
    for(const auto& search_method : search_methods) {
      std::cout << "--- pclomp::NDT (" << search_method.first << ", " << n << " threads) ---" << std::endl;
      ndt_omp->setNumThreads(n);
      ndt_omp->setNeighborhoodSearchMethod(search_method.second);
      aligned = align(ndt_omp, target_cloud_xyz, source_cloud_xyz);
    }
  }

  // visulization
  pcl::visualization::PCLVisualizer vis("vis");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_handler(target_cloud_xyz, 255.0, 0.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_handler(source_cloud_xyz, 0.0, 255.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligned_handler(aligned_viz, 0.0, 0.0, 255.0);
  vis.addPointCloud(target_cloud_xyz, target_handler, "target");
  vis.addPointCloud(source_cloud_xyz, source_handler, "source");
  vis.addPointCloud(aligned_viz, aligned_handler, "aligned");
  // add text to the viewer with colors
  vis.addText("target", 10, 10, 12, 1.0, 0.0, 0.0, "target_text");
  vis.addText("source", 10, 30, 12, 0.0, 1.0, 0.0, "source_text");
  vis.addText("aligned", 10, 50, 12, 0.0, 0.0, 1.0, "aligned_text");
  vis.setPosition(0, 0);
  vis.spin();

  return 0;
}
