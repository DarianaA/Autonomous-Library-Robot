#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include "pcl_manipulation_server.h"
#include "segmentation.h"
#include "downsample.h"
#include "euclidian.h"
#include "cartesian_filter.h"
#include "min_max.h"
#include "centroid.h"
#include <combine_clouds.h>

int main (int argc, char **argv)
{

    ros::init(argc, argv, "pcl_manipulation");
    ros::Time::init();
    ros::Duration(2).sleep();

    ros::NodeHandle n;
    ros::ServiceServer segment_plane_service = n.advertiseService("/pcl_manipulation/segment_plane", handle_segment_plane);
    ros::ServiceServer remove_outliers_service = n.advertiseService("/pcl_manipulation/remove_outliers", handle_remove_outliers);
    ros::ServiceServer downsample_service = n.advertiseService("/pcl_manipulation/downsample", handle_downsample);
    ros::ServiceServer euclidian_clustering_service = n.advertiseService("/pcl_manipulation/cluster", handle_euclidian_clustering);
    ros::ServiceServer cartesian_filtering_service = n.advertiseService("/pcl_manipulation/cartesian_filter", handle_cartesian_filter);
    ros::ServiceServer min_max_service = n.advertiseService("/pcl_manipulation/min_max", handle_min_max);
    ros::ServiceServer centroid_service = n.advertiseService("/pcl_manipulation/centroid", handle_centroid);
    ros::ServiceServer combine_clouds_service = n.advertiseService("/pcl_manipulation/combine_clouds", handle_combine_clouds);

    ros::spin();
    return 0;
}

bool handle_segment_plane (pcl_manipulation::SegmentPlane::Request &req,
                          pcl_manipulation::SegmentPlane::Response &res)
{   
    std::cout << "Was updated" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // create a Point Cloud from the input and remove NaNs
    pcl::fromROSMsg(req.points, *cloud);
    segment_plane(cloud, plane_filtered_cloud, req.distance_threshold);

    pcl::toROSMsg(*plane_filtered_cloud, res.points);
    return true;
}

bool handle_remove_outliers(pcl_manipulation::RemoveOutliers::Request &req,
                            pcl_manipulation::RemoveOutliers::Response &res)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_outliers(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(req.points, *cloud);
    remove_outliers(cloud, cloud_without_outliers, req.neighbours, req.stddev, req.iterations);
    pcl::toROSMsg(*cloud_without_outliers, res.points);
    return true;

}

bool handle_downsample(pcl_manipulation::Downsample::Request &req,
                            pcl_manipulation::Downsample::Response &res)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);   

    pcl::fromROSMsg(req.points, *cloud);
    downsample_voxel(cloud, downsampled_cloud, req.leaf_size_x, req.leaf_size_y, req.leaf_size_z);
    pcl::toROSMsg(*downsampled_cloud, res.points);
    return true;

}

bool handle_euclidian_clustering(pcl_manipulation::Euclidian::Request &req, pcl_manipulation::Euclidian::Response &res) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(req.points, *cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    euclidian_clustering(cloud, cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it!= cluster_indices.end(); it++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        cloud_cluster->header.stamp = cloud->header.stamp;
        cloud_cluster->header.frame_id = cloud->header.frame_id;

        for (const auto& idx : it->indices)
            cloud_cluster->push_back ((*cloud)[idx]);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        sensor_msgs::PointCloud2 pcl2;
        pcl::toROSMsg(*cloud_cluster, pcl2);
        res.clusters.push_back(pcl2);
    }

    return true;
}

bool handle_cartesian_filter(pcl_manipulation::CartesianFilter::Request &req, pcl_manipulation::CartesianFilter::Response &res) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(req.points, *cloud);

    std::vector<std::string> axes{"x", "y", "z"};

    if (std::find(std::begin(axes), std::end(axes), req.axis.data) == std::end(axes))
        return false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cartesian_filter(cloud, filtered_cloud, req.axis.data, req.lower, req.upper);

    pcl::toROSMsg(*filtered_cloud, res.points);

    return true;
}

bool handle_min_max(pcl_manipulation::MinMax::Request &req, pcl_manipulation::MinMax::Response &res) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(req.points, *cloud);

    std::vector<pcl::PointXYZ> bounds;

    min_max(cloud, bounds);

    for (const auto& idx : bounds) {
        std::cout << "x " << idx.x << " y " << idx.y << " z " << idx.z << std::endl;
    }
    res.min_x = bounds[0].x;
    res.min_y = bounds[0].y;
    res.min_z = bounds[0].z;
    res.max_x = bounds[1].x;
    res.max_y = bounds[1].y;
    res.max_z = bounds[1].z;

    return true;

}

bool handle_centroid(pcl_manipulation::Centroid::Request &req, pcl_manipulation::Centroid::Response &res) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(req.points, *cloud);

    pcl::PointXYZ centroid_;

    centroid(cloud, centroid_);

    geometry_msgs::PointStamped pt;
    pt.header = req.points.header;
    pt.point.x = centroid_.x;
    pt.point.y = centroid_.y;
    pt.point.z = centroid_.z;

    return true;

}

bool handle_combine_clouds(pcl_manipulation::CombineClouds::Request &req, pcl_manipulation::CombineClouds::Response &res) {

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(req.points[0], *cloud);

    bool first = true;
    for (auto const c : req.points) {
        if (first) {first = false; continue;}
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(c, *cloud_);
        clouds.push_back(cloud_);
    }

    combine_clouds(clouds, cloud);

    pcl::toROSMsg(*cloud, res.points);

    return true;
}


