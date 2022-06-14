#ifndef CARTESIAN_FILTER_H
#define CARTESIAN_FILTER_H

#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

void cartesian_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud, std::string axis, float lower, float upper);

#endif

