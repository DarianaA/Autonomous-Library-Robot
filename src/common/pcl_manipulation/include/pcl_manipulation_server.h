#ifndef PCL_MANIPULATION_SERVER_H
#define PCL_MANIPULATION_SERVER_H
#include <iostream>
#include <vector>
#include <string>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

#include "pcl_manipulation/SegmentPlane.h"
#include "pcl_manipulation/RemoveOutliers.h"
#include "pcl_manipulation/Downsample.h"
#include "pcl_manipulation/Crop.h"
#include "pcl_manipulation/Euclidian.h"
#include "pcl_manipulation/CartesianFilter.h"
#include "pcl_manipulation/MinMax.h"
#include "pcl_manipulation/Centroid.h"
#include "pcl_manipulation/CombineClouds.h"

bool handle_segment_plane(pcl_manipulation::SegmentPlane::Request &req, pcl_manipulation::SegmentPlane::Response &res);
bool handle_remove_outliers(pcl_manipulation::RemoveOutliers::Request &req, pcl_manipulation::RemoveOutliers::Response &res);
bool handle_downsample(pcl_manipulation::Downsample::Request &req, pcl_manipulation::Downsample::Response &res);
bool handle_euclidian_clustering(pcl_manipulation::Euclidian::Request &req, pcl_manipulation::Euclidian::Response &res);
bool handle_cartesian_filter(pcl_manipulation::CartesianFilter::Request &req, pcl_manipulation::CartesianFilter::Response &res);
bool handle_min_max(pcl_manipulation::MinMax::Request &req, pcl_manipulation::MinMax::Response &res);
bool handle_centroid(pcl_manipulation::Centroid::Request &req, pcl_manipulation::Centroid::Response &res);
bool handle_combine_clouds(pcl_manipulation::CombineClouds::Request &req, pcl_manipulation::CombineClouds::Response &res);

#endif