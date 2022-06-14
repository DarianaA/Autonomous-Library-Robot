#include "combine_clouds.h"

void combine_clouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    for (auto const&c : clouds)
        *cloud += *c;
}