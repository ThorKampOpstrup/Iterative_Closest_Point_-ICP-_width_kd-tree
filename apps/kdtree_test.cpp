#include "kdtree.h"

#include "iostream"

#define DATA_DIMENSION 5
#define N 100
#define KD_DIM 3

int main() {
  std::cout << "Hello World" << std::endl;
  // return 0;
  kdtree tree;
  kdtree_allocate(&tree, KD_DIM, N);

  // make a list of all data
  float data_list[6][DATA_DIMENSION] = {{-1.0, -1.0, -1.0, 0.0, 0.0}, {1.0, 1.0, 1.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0}, {0.5, 0.5, 0.5, 0.0, 0.0}, {0.25, 0.25, 0.25, 0.0, 0.0}, {0.0, 0.75, 0.75, 0.0, 0.0}};

  kdtree_insert(tree, data_list, 6);

  float test_point[DATA_DIMENSION] = {0, -1, 1.0, 0.01};

  std::cout << std::endl << "Printing all nodes in the tree: " << std::endl;

  kdtree_print_all_nodes(tree);
  std::cout << std::endl;

  // //print out all distances to points in the tree
  std::cout << std::endl << "Printing all distances to points in the tree: " << std::endl;
  float lowest_distance, distance;
  for (int i = 0; i < tree.number_of_nodes; i++) {
    kdtree_node* node = tree.root + i;
    distance = kdtree_distance(tree, test_point, node->data);
    if(i == 0){
      lowest_distance = distance;
    }
    if (distance < lowest_distance) {
      lowest_distance = distance;
    }
    std::cout << "Distance to point " << i << ": " << distance << std::endl;
  }
  std::cout << "Lowest distance found to be: " << lowest_distance << std::endl;
 
 
  kdtree_node* closest_point = kdtree_NN(tree, test_point);

  std::cout << "Closest point to test point: ";
  kdtree_print_node_info(tree, closest_point);
  std::cout << "distance: " << kdtree_distance(tree, test_point, closest_point->data) << std::endl;

  return 0;
}
