#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <ctime>

#include <nice_global_planner/KDTree.hpp>

int
main (int argc, char** argv)
{
  srand (time (NULL));

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  KDTree<double> kdTree(3);

  // Generate pointcloud data
  cloud->width = 1000;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
    std::vector<double> data;
    data.resize(3);
    data[0] = cloud->points[i].x;
    data[1] = cloud->points[i].y;
    data[2] = cloud->points[i].z;
    kdTree.insert(data);
    assert(kdTree.findNode(data) != NULL);
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  kdtree.setInputCloud (cloud);

  pcl::PointXYZ searchPoint;

  searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
  searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
  searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

  std::vector<double> searchVector;
  searchVector.resize(3);
  searchVector[0] = searchPoint.x;
  searchVector[1] = searchPoint.y;
  searchVector[2] = searchPoint.z;


  KDNode<double> *nearest = kdTree.nearestNeighbour(searchVector);

  std::cout << " nearestneighbour[0]: " << (*nearest).data[0] << std::endl; 
  std::cout << " nearestneighbour[1]: " << (*nearest).data[1] << std::endl;
  std::cout << " nearestneighbour[2]: " << (*nearest).data[2] << std::endl;

  // K nearest neighbor search

  int K = 1;

  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  std::cout << "K nearest neighbor search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with K=" << K << std::endl;

  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
      std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
                << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
                << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
                << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
  }

  // Neighbors within radius search

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

  // std::cout << "Neighbors within radius search at (" << searchPoint.x 
  //           << " " << searchPoint.y 
  //           << " " << searchPoint.z
  //           << ") with radius=" << radius << std::endl;


  // if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
  // {
  //   for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
  //     std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
  //               << " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
  //               << " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
  //               << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
  // }


  return 0;
}