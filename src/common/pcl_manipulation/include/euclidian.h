#ifndef EUCLIDIAN_H
#define EUCLIDIAN_H

#include <vector>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

void euclidian_clustering (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointIndices>& clusters);

#endif

