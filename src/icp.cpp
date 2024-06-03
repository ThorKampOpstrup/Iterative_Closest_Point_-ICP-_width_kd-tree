#include <iostream>
#include <numeric>
#include "icp.h"
#include "Eigen/Eigen"
#include "chrono"

// #include "kdtree.hpp"

using namespace std;
using namespace Eigen;

#define PRINT_TIMINGS 1

// int allocate_icp

Eigen::MatrixXd pcl_to_eigen(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) {
  uint n = pc->width * pc->height;
  Eigen::MatrixXd mat = Eigen::MatrixXd::Ones(n, 3);
  for(int i = 0; i < n; i++) {
    mat(i, 0) = pc->points[i].x;
    mat(i, 1) = pc->points[i].y;
    mat(i, 2) = pc->points[i].z;
  }
  return mat;
};

Eigen::MatrixXd pcl_to_eigen(pcl::PointCloud<pcl::PointXYZI>::Ptr pc) {
  uint n = pc->width * pc->height;
  Eigen::MatrixXd mat = Eigen::MatrixXd::Ones(n, 3);
  for(int i = 0; i < n; i++) {
    mat(i, 0) = pc->points[i].x;
    mat(i, 1) = pc->points[i].y;
    mat(i, 2) = pc->points[i].z;
  }
  return mat;
};

pcl::PointCloud<pcl::PointXYZ>::Ptr eigen_to_pcl(Eigen::MatrixXd &matrix) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
  int row = matrix.rows();
  pc->width = row;
  pc->height = 1;
  pc->points.resize(row);
  for(int i = 0; i < row; i++) {
    pc->points[i].x = matrix(i, 0);
    pc->points[i].y = matrix(i, 1);
    pc->points[i].z = matrix(i, 2);
  }
  return pc;
};

Eigen::Matrix4d best_fit_transform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B) {
  /*
  Notice:
  1/ JacobiSVD return U,S,V, S as a vector, "use U*S*Vt" to get original Matrix;
  2/ matrix type 'MatrixXd' or 'MatrixXf' matters.
  */
  Eigen::Matrix4d T = Eigen::MatrixXd::Identity(4, 4);
  Eigen::Vector3d centroid_A(0, 0, 0);
  Eigen::Vector3d centroid_B(0, 0, 0);
  Eigen::MatrixXd AA = A;
  Eigen::MatrixXd BB = B;
  int row = A.rows();

  for(int i = 0; i < row; i++) {
    centroid_A += A.block<1, 3>(i, 0).transpose();
    centroid_B += B.block<1, 3>(i, 0).transpose();
  }
  centroid_A /= row;
  centroid_B /= row;
  for(int i = 0; i < row; i++) {
    AA.block<1, 3>(i, 0) = A.block<1, 3>(i, 0) - centroid_A.transpose();
    BB.block<1, 3>(i, 0) = B.block<1, 3>(i, 0) - centroid_B.transpose();
  }

  Eigen::MatrixXd H = AA.transpose() * BB;
  Eigen::MatrixXd U;
  Eigen::VectorXd S;
  Eigen::MatrixXd V;
  Eigen::MatrixXd Vt;
  Eigen::Matrix3d R;
  Eigen::Vector3d t;

  JacobiSVD<Eigen::MatrixXd> svd(H, ComputeFullU | ComputeFullV);
  U = svd.matrixU();
  S = svd.singularValues();
  V = svd.matrixV();
  Vt = V.transpose();

  R = Vt.transpose() * U.transpose();

  if(R.determinant() < 0) {
    Vt.block<1, 3>(2, 0) *= -1;
    R = Vt.transpose() * U.transpose();
  }

  t = centroid_B - R * centroid_A;

  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;
  return T;
}

int _icp_allocate_mem(uint src_row, uint dst_row, ICP_OUT *result, NEIGHBOR *neighbor) {
  result->distances = (float *)malloc(sizeof(float) * src_row);
  if(result->distances == NULL) {
    std::cerr << "failed to allocate memory for distances" << std::endl;
    return 0;
  }
  neighbor->distances = (float *)malloc(sizeof(float) * src_row);
  if(neighbor->distances == NULL) {
    std::cerr << "failed to allocate memory for distances" << std::endl;
    std::free(result->distances);
    return 0;
  }
  neighbor->indices = (float *)malloc(sizeof(float) * src_row);
  if(neighbor->indices == NULL) {
    std::cerr << "failed to allocate memory for indices" << std::endl;
    std::free(result->distances);
    std::free(neighbor->distances);
    return 0;
  }
  return 1;
}

void _icp_free_mem(ICP_OUT *result, NEIGHBOR *neighbor) {
  std::free(result->distances);
  std::free(neighbor->distances);
  std::free(neighbor->indices);
}

void _calc_mean_error(float *vec, int size, double &mean) {
  mean = std::accumulate(vec, vec + size, 0.0) / size;
}

// Function to compute the angular error between two rotation matrices
double compute_rot_error(const Matrix3d &R1, const Matrix3d &R2) {
  Matrix3d R_error = R1.transpose() * R2;
  double trace_R_error = R_error.trace();
  double angular_error = acos((trace_R_error - 1.0) / 2.0);  // in radians
  return angular_error;
}

// Function to compute the translational error between two translation vectors
double compute_trans_error(const Vector3d &t1, const Vector3d &t2) {
  return (t1 - t2).norm();
}

// Main function to compute mean angular and translational error between two transformation matrices
void compute_mean_error_gt(const Matrix4d &T1, const Matrix4d &T2) {
  // Extract rotation matrices
  Matrix3d R1 = T1.block<3, 3>(0, 0);
  Matrix3d R2 = T2.block<3, 3>(0, 0);

  // Extract translation vectors
  Vector3d t1 = T1.block<3, 1>(0, 3);
  Vector3d t2 = T2.block<3, 1>(0, 3);

  // Compute errors
  double angular_error = compute_rot_error(R1, R2);
  double translational_error = compute_trans_error(t1, t2);

  // Convert angular error to degrees for better interpretability
  double angular_error_degrees = angular_error * (180.0 / M_PI);

  if(isnan(angular_error_degrees)){
    angular_error_degrees = 0;
  }
  if(isnan(translational_error)){
    translational_error = 0;
  }

  std::cout << angular_error_degrees << ", " << translational_error << std::endl;
}

ICP_OUT icp_convergence(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, int max_iterations, int tolerance, kdtree *kd_tree, Eigen::Matrix4d &gt_trans) {
  ICP_OUT result;
  NEIGHBOR neighbor;
  if(!_icp_allocate_mem(A.rows(), B.rows(), &result, &neighbor)) {
    return result;
  }

  int row = A.rows();
  Eigen::MatrixXd src = Eigen::MatrixXd::Ones(3 + 1, row);
  Eigen::MatrixXd src3d = Eigen::MatrixXd::Ones(3, row);
  Eigen::MatrixXd dst = Eigen::MatrixXd::Ones(3 + 1, row);
  Eigen::Matrix4d T;
  Eigen::MatrixXd dst_chorder = Eigen::MatrixXd::Ones(3, row);
  int iter = 0;

#if PRINT_TIMINGS
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = std::chrono::high_resolution_clock::now();
#endif

  for(int i = 0; i < row; i++) {
    src.block<3, 1>(0, i) = A.block<1, 3>(i, 0).transpose();
    src3d.block<3, 1>(0, i) = A.block<1, 3>(i, 0).transpose();
    dst.block<3, 1>(0, i) = B.block<1, 3>(i, 0).transpose();
  }

  double prev_error = 0;
  double mean_error = 0;
  for(int i = 0; i < max_iterations; i++) {
#if PRINT_TIMINGS
    t1 = std::chrono::high_resolution_clock::now();
#endif
    // neighbor = nearest_neighbor(src3d.transpose(), B);
    nearest_neighbor(src3d.transpose(), B, kd_tree, neighbor);

#if PRINT_TIMINGS
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "nearest_neighbor time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "micro seconds" << std::endl;
#endif

#if PRINT_TIMINGS
    t1 = std::chrono::high_resolution_clock::now();
#endif
    for(int j = 0; j < row; j++) {
      dst_chorder.block<3, 1>(0, j) = dst.block<3, 1>(0, neighbor.indices[j]);
    }

#if PRINT_TIMINGS
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "dst_chorder time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "micro seconds" << std::endl;
#endif

#if PRINT_TIMINGS
    t1 = std::chrono::high_resolution_clock::now();
#endif
    T = best_fit_transform(src3d.transpose(), dst_chorder.transpose());

#if PRINT_TIMINGS
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "best_fit_transform time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "micro seconds" << std::endl;
#endif

#if PRINT_TIMINGS
    t1 = std::chrono::high_resolution_clock::now();
#endif
    src = T * src;
    for(int j = 0; j < row; j++) {
      src3d.block<3, 1>(0, j) = src.block<3, 1>(0, j);
    }

#if PRINT_TIMINGS
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "src time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "micro seconds" << std::endl;
#endif

#if PRINT_TIMINGS
    t1 = std::chrono::high_resolution_clock::now();
#endif

    auto tmp_trans = best_fit_transform(A, src3d.transpose());
    compute_mean_error_gt(tmp_trans, gt_trans);

    _calc_mean_error(neighbor.distances, row, mean_error);
    if(abs(prev_error - mean_error) < tolerance) {
      break;
    }
    prev_error = mean_error;
    iter = i + 2;

#if PRINT_TIMINGS
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "mean_error time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "micro seconds" << std::endl;
#endif
  }
  T = best_fit_transform(A, src3d.transpose());
  result.trans = T;
  result.distances = neighbor.distances;
  result.iter = iter;

  return result;
}

ICP_OUT icp_convergence(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, int max_iterations, int tolerance, Eigen::Matrix4d &gt_trans) {
  NEIGHBOR neighbor;
  ICP_OUT result;
  if(!_icp_allocate_mem(A.rows(), B.rows(), &result, &neighbor)) {
    return result;
  }

  int row = A.rows();
  Eigen::MatrixXd src = Eigen::MatrixXd::Ones(3 + 1, row);
  Eigen::MatrixXd src3d = Eigen::MatrixXd::Ones(3, row);
  Eigen::MatrixXd dst = Eigen::MatrixXd::Ones(3 + 1, row);
  Eigen::Matrix4d T;
  Eigen::MatrixXd dst_chorder = Eigen::MatrixXd::Ones(3, row);
  int iter = 0;

#if PRINT_TIMINGS
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = std::chrono::high_resolution_clock::now();
#endif

  for(int i = 0; i < row; i++) {
    src.block<3, 1>(0, i) = A.block<1, 3>(i, 0).transpose();
    src3d.block<3, 1>(0, i) = A.block<1, 3>(i, 0).transpose();
    dst.block<3, 1>(0, i) = B.block<1, 3>(i, 0).transpose();
  }

  double prev_error = 0;
  double mean_error = 0;
  for(int i = 0; i < max_iterations; i++) {
#if PRINT_TIMINGS
    t1 = std::chrono::high_resolution_clock::now();
#endif
    nearest_neighbor(src3d.transpose(), B, neighbor);  //! This should be called as a ros service

#if PRINT_TIMINGS
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "nearest_neighbor time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "micro seconds" << std::endl;
#endif

#if PRINT_TIMINGS
    t1 = std::chrono::high_resolution_clock::now();
#endif
    for(int j = 0; j < row; j++) {
      dst_chorder.block<3, 1>(0, j) = dst.block<3, 1>(0, neighbor.indices[j]);
    }

#if PRINT_TIMINGS
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "dst_chorder time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "micro seconds" << std::endl;
#endif

#if PRINT_TIMINGS
    t1 = std::chrono::high_resolution_clock::now();
#endif
    T = best_fit_transform(src3d.transpose(), dst_chorder.transpose());

#if PRINT_TIMINGS
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "best_fit_transform time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "micro seconds" << std::endl;
#endif

#if PRINT_TIMINGS
    t1 = std::chrono::high_resolution_clock::now();
#endif
    src = T * src;
    for(int j = 0; j < row; j++) {
      src3d.block<3, 1>(0, j) = src.block<3, 1>(0, j);
    }

#if PRINT_TIMINGS
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "src time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "micro seconds" << std::endl;
#endif

#if PRINT_TIMINGS
    t1 = std::chrono::high_resolution_clock::now();
#endif

    auto tmp_trans = best_fit_transform(A, src3d.transpose());
    compute_mean_error_gt(tmp_trans, gt_trans);

    _calc_mean_error(neighbor.distances, row, mean_error);
    if(abs(prev_error - mean_error) < tolerance) {
      break;
    }
    prev_error = mean_error;
    iter = i + 2;

#if PRINT_TIMINGS
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "mean_error time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "micro seconds" << std::endl;
#endif
  }

  T = best_fit_transform(A, src3d.transpose());
  result.trans = T;
  result.distances = neighbor.distances;
  result.iter = iter;

  return result;
}

// kd tree method of nn
ICP_OUT icp(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, int max_iterations, int tolerance, kdtree *kd_tree) {
  ICP_OUT result;
  NEIGHBOR neighbor;
  if(!_icp_allocate_mem(A.rows(), B.rows(), &result, &neighbor)) {
    return result;
  }

  int row = A.rows();
  Eigen::MatrixXd src = Eigen::MatrixXd::Ones(3 + 1, row);
  Eigen::MatrixXd src3d = Eigen::MatrixXd::Ones(3, row);
  Eigen::MatrixXd dst = Eigen::MatrixXd::Ones(3 + 1, row);
  Eigen::Matrix4d T;
  Eigen::MatrixXd dst_chorder = Eigen::MatrixXd::Ones(3, row);
  int iter = 0;

#if PRINT_TIMINGS
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = std::chrono::high_resolution_clock::now();
#endif

  for(int i = 0; i < row; i++) {
    src.block<3, 1>(0, i) = A.block<1, 3>(i, 0).transpose();
    src3d.block<3, 1>(0, i) = A.block<1, 3>(i, 0).transpose();
    dst.block<3, 1>(0, i) = B.block<1, 3>(i, 0).transpose();
  }

  double prev_error = 0;
  double mean_error = 0;
  for(int i = 0; i < max_iterations; i++) {
#if PRINT_TIMINGS
    t1 = std::chrono::high_resolution_clock::now();
#endif
    // neighbor = nearest_neighbor(src3d.transpose(), B);
    nearest_neighbor(src3d.transpose(), B, kd_tree, neighbor);

#if PRINT_TIMINGS
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "nearest_neighbor time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "micro seconds" << std::endl;
#endif

#if PRINT_TIMINGS
    t1 = std::chrono::high_resolution_clock::now();
#endif
    for(int j = 0; j < row; j++) {
      dst_chorder.block<3, 1>(0, j) = dst.block<3, 1>(0, neighbor.indices[j]);
    }

#if PRINT_TIMINGS
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "dst_chorder time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "micro seconds" << std::endl;
#endif

#if PRINT_TIMINGS
    t1 = std::chrono::high_resolution_clock::now();
#endif
    T = best_fit_transform(src3d.transpose(), dst_chorder.transpose());

#if PRINT_TIMINGS
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "best_fit_transform time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "micro seconds" << std::endl;
#endif

#if PRINT_TIMINGS
    t1 = std::chrono::high_resolution_clock::now();
#endif
    src = T * src;
    for(int j = 0; j < row; j++) {
      src3d.block<3, 1>(0, j) = src.block<3, 1>(0, j);
    }

#if PRINT_TIMINGS
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "src time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "micro seconds" << std::endl;
#endif

#if PRINT_TIMINGS
    t1 = std::chrono::high_resolution_clock::now();
#endif
    _calc_mean_error(neighbor.distances, row, mean_error);
    if(abs(prev_error - mean_error) < tolerance) {
      break;
    }
    // std::cout << "t: \n\r" << T.block<3, 1>(0, 3) << std::endl;
    prev_error = mean_error;
    iter = i + 2;

#if PRINT_TIMINGS
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "mean_error time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "micro seconds" << std::endl;
#endif
  }
  T = best_fit_transform(A, src3d.transpose());
  // T.block<3, 3>(0, 0) = T.block<3, 3>(0, 0) / T.block<3, 3>(0, 0).norm();
  result.trans = T;
  result.distances = neighbor.distances;
  result.iter = iter;

  return result;
}

// dumb and stupid use and implementation of nn
ICP_OUT icp(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, int max_iterations, int tolerance) {
  NEIGHBOR neighbor;
  ICP_OUT result;
  if(!_icp_allocate_mem(A.rows(), B.rows(), &result, &neighbor)) {
    return result;
  }

  int row = A.rows();
  Eigen::MatrixXd src = Eigen::MatrixXd::Ones(3 + 1, row);
  Eigen::MatrixXd src3d = Eigen::MatrixXd::Ones(3, row);
  Eigen::MatrixXd dst = Eigen::MatrixXd::Ones(3 + 1, row);
  Eigen::Matrix4d T;
  Eigen::MatrixXd dst_chorder = Eigen::MatrixXd::Ones(3, row);
  int iter = 0;

#if PRINT_TIMINGS
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = std::chrono::high_resolution_clock::now();
#endif

  for(int i = 0; i < row; i++) {
    src.block<3, 1>(0, i) = A.block<1, 3>(i, 0).transpose();
    src3d.block<3, 1>(0, i) = A.block<1, 3>(i, 0).transpose();
    dst.block<3, 1>(0, i) = B.block<1, 3>(i, 0).transpose();
  }

  double prev_error = 0;
  double mean_error = 0;
  for(int i = 0; i < max_iterations; i++) {
#if PRINT_TIMINGS
    t1 = std::chrono::high_resolution_clock::now();
#endif
    nearest_neighbor(src3d.transpose(), B, neighbor);  //! This should be called as a ros service

#if PRINT_TIMINGS
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "nearest_neighbor time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "micro seconds" << std::endl;
#endif

#if PRINT_TIMINGS
    t1 = std::chrono::high_resolution_clock::now();
#endif
    for(int j = 0; j < row; j++) {
      dst_chorder.block<3, 1>(0, j) = dst.block<3, 1>(0, neighbor.indices[j]);
    }

#if PRINT_TIMINGS
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "dst_chorder time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "micro seconds" << std::endl;
#endif

#if PRINT_TIMINGS
    t1 = std::chrono::high_resolution_clock::now();
#endif
    T = best_fit_transform(src3d.transpose(), dst_chorder.transpose());

#if PRINT_TIMINGS
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "best_fit_transform time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "micro seconds" << std::endl;
#endif

#if PRINT_TIMINGS
    t1 = std::chrono::high_resolution_clock::now();
#endif
    src = T * src;
    for(int j = 0; j < row; j++) {
      src3d.block<3, 1>(0, j) = src.block<3, 1>(0, j);
    }

#if PRINT_TIMINGS
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "src time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "micro seconds" << std::endl;
#endif

#if PRINT_TIMINGS
    t1 = std::chrono::high_resolution_clock::now();
#endif
    _calc_mean_error(neighbor.distances, row, mean_error);
    if(abs(prev_error - mean_error) < tolerance) {
      break;
    }
    prev_error = mean_error;
    iter = i + 2;

#if PRINT_TIMINGS
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "mean_error time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "micro seconds" << std::endl;
#endif
  }

  T = best_fit_transform(A, src3d.transpose());
  result.trans = T;
  result.distances = neighbor.distances;
  result.iter = iter;

  return result;
}

void nearest_neighbor(const Eigen::MatrixXd &src, const Eigen::MatrixXd &dst, NEIGHBOR &neigh) {
  int row_src = src.rows();
  int row_dst = dst.rows();
  Eigen::Vector3d vec_src;
  Eigen::Vector3d vec_dst;
  // NEIGHBOR neigh;
  float min = 100;
  int index = 0;
  float dist_temp = 0;

  for(int ii = 0; ii < row_src; ii++) {
    vec_src = src.block<1, 3>(ii, 0).transpose();
    min = 100;
    index = 0;
    dist_temp = 0;
    for(int jj = 0; jj < row_dst; jj++) {
      vec_dst = dst.block<1, 3>(jj, 0).transpose();
      dist_temp = dist(vec_src, vec_dst);
      if(dist_temp < min) {
        min = dist_temp;
        index = jj;
      }
    }
    neigh.distances[ii] = min;
    neigh.indices[ii] = index;
  }

  // return neigh;
}

void nearest_neighbor(const Eigen::MatrixXd &src, const Eigen::MatrixXd &dst, kdtree *tree, NEIGHBOR &neigh) {
  int row_src = src.rows();
  int row_dst = dst.rows();
  Eigen::Vector3d vec_src[row_src];
  Eigen::Vector3d vec_dst[row_dst];

  float tmp_point[3] = {0, 0, 0};
  for(int ii = 0; ii < row_src; ii++) {
    tmp_point[0] = src(ii, 0);
    tmp_point[1] = src(ii, 1);
    tmp_point[2] = src(ii, 2);

    auto t1 = std::chrono::high_resolution_clock::now();
    kdtree_node *nn_node = kdtree_NN(*tree, tmp_point);
    auto t2 = std::chrono::high_resolution_clock::now();
    int duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000;
    if(duration != 0) {
      std::cout << "******************kdtree_NN time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "micro seconds" << std::endl;
    }

    t1 = std::chrono::high_resolution_clock::now();
    neigh.distances[ii] = sqrt(distance_squared(*tree, nn_node->data, tmp_point));
    neigh.indices[ii] = nn_node->data[ORIGINAL_INDEX_index];
  }

}

float dist(const Eigen::Vector3d &pta, const Eigen::Vector3d &ptb) {
  return sqrt((pta[0] - ptb[0]) * (pta[0] - ptb[0]) + (pta[1] - ptb[1]) * (pta[1] - ptb[1]) + (pta[2] - ptb[2]) * (pta[2] - ptb[2]));
}
