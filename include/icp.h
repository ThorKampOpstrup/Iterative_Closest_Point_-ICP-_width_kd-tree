#include "Eigen/Eigen"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <vector>
#include "kdtree.h"

#ifndef ICP_H
#define ICP_H


#define PRINT_TIMINGS 0

typedef struct{
    Eigen::Matrix4d trans;
    float *distances;
    int iter;
}  ICP_OUT;

typedef struct{
    float *distances;
    float *indices;
} NEIGHBOR;

Eigen::Matrix4d best_fit_transform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);

ICP_OUT icp_convergence(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, int max_iterations, int tolerance, kdtree *tree, Eigen::Matrix4d &gt_trans);
ICP_OUT icp_convergence(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, int max_iterations, int tolerance, Eigen::Matrix4d &gt_trans);

ICP_OUT icp(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, int max_iterations, int tolerance, kdtree *tree);
ICP_OUT icp(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, int max_iterations, int tolerance);



// helper functions
int _icp_allocate_mem(uint src_row, uint dst_row, ICP_OUT *result, NEIGHBOR *neighbor);
void _icp_free_mem(ICP_OUT *result, NEIGHBOR *neighbor);

void _calc_mean_error(float *vec, int size, double &mean);
// throughout method
void nearest_neighbor(const Eigen::MatrixXd &src, const Eigen::MatrixXd &dst, NEIGHBOR &neighbor);
void nearest_neighbor(const Eigen::MatrixXd &src, const Eigen::MatrixXd &dst, kdtree *tree, NEIGHBOR &neighbor);

float dist(const Eigen::Vector3d &pta, const Eigen::Vector3d &ptb);

Eigen::MatrixXd pcl_to_eigen(pcl::PointCloud<pcl::PointXYZ>::Ptr pc);
Eigen::MatrixXd pcl_to_eigen(pcl::PointCloud<pcl::PointXYZI>::Ptr pc);

pcl::PointCloud<pcl::PointXYZ>::Ptr eigen_to_pcl(Eigen::MatrixXd &matrix);

#endif
