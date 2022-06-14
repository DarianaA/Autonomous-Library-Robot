#include "min_max.h"

void min_max(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointXYZ> &bounds) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr nan_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud(*cloud, *nan_filtered_cloud, nan_indices);
    cloud.swap(nan_filtered_cloud);

    pcl::PointXYZ min, max;
    pcl::getMinMax3D (*cloud, min, max);
    bounds.push_back(min);
    bounds.push_back(max);

}
