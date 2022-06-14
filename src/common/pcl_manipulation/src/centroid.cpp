#include "centroid.h"

void centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ &centroid) {

    pcl::CentroidPoint<pcl::PointXYZ> centroid_point;
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud->begin(); it != cloud->end(); it++) {
        centroid_point.add(*it);
    }
    centroid_point.get(centroid);

}
