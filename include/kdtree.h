// this file will generate a basic kd tree for the point cloud based on the xyz positions of the points

#include <stdio.h>
#include <stdlib.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#ifndef KDTREE_H
#define KDTREE_H

#define DATA_DIMENSION 5

#define X_index 0
#define Y_index 1
#define Z_index 2
#define INTENSITY_index 3
#define ORIGINAL_INDEX_index 4

typedef struct kdtree_node {
  // struct kdtree_node *left, *right, *parent;
  uint left_index = 0, right_index = 0, parent_index = 0;
  float data[DATA_DIMENSION]; //Should be changed to Data type
} kdtree_node;



typedef struct kdtree {
  kdtree_node *root;
  uint dim;
  uint number_of_nodes = 0;  // number of nodes inserted in the tree
} kdtree;

// take a number of expected nodes and return 1 if succes and 0 if fail.
// dim is the number of dimensions of each point, that they shout be sorted for
// n is the number of nodes that the tree should be able to hold
int kdtree_allocate(kdtree *tree, uint dim_in, uint n);

// Insert a point into the tree, a point is a 3d point(can contain intensities or other data)
void add_node(kdtree &tree, float data[DATA_DIMENSION]);

// Insert an array of points into the tree, an array of points is a set of 3d points(can contain intensities or other data)
// Points = [[],[],...,[]] where each [] is a 3d point
// n is the number of points
// The point will be added from the middle of the points array
void kdtree_insert(kdtree &tree, float points[][DATA_DIMENSION], uint n);
void kdtree_insert(kdtree &tree, pcl::PointCloud<pcl::PointXYZI>::Ptr pc);
void kdtree_insert(kdtree &tree, pcl::PointCloud<pcl::PointXYZ>::Ptr pc);

float distance_squared(kdtree &tree, float a_data[DATA_DIMENSION], float b_data[DATA_DIMENSION]);
float kdtree_distance(kdtree &tree, float a_data[DATA_DIMENSION], float b_data[DATA_DIMENSION]);

kdtree_node *kdtree_NN(kdtree &tree, float target[DATA_DIMENSION]);
kdtree_node *kdtree_NN_intern(kdtree &tree, float target[DATA_DIMENSION], uint starting_index, uint dimension_to_compare);


void kdtree_print_node_info(kdtree &tree, uint index);
void kdtree_print_node_info(kdtree &tree, kdtree_node *node);
void kdtree_print_depth(kdtree &tree);
void kdtree_print_all_nodes(kdtree &tree);

void kdtree_free(kdtree *tree);

#endif /* KDTREE_H */