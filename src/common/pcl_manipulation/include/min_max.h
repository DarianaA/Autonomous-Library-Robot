#ifndef MIN_MAX_H
#define MIN_MAX_H

#include <vector>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>

void min_max(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointXYZ> &bounds);

#endif