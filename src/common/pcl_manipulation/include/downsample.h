#ifndef DOWNSAMPLE_H
#define DOWNSAMPLE_H

#include <pcl/filters/voxel_grid.h>

void downsample_voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud, float leaf_size_x, float leaf_size_y, float leaf_size_z);

#endif