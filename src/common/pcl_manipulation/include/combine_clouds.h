#ifndef COMBINE_CLOUDS_H
#define COMBINE_CLOUDS_H

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

void combine_clouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >& clouds, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

#endif