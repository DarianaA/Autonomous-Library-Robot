#ifndef CENTROID_H
#define CENTROID_H

#include <pcl/common/centroid.h>

void centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ &centroid);

#endif