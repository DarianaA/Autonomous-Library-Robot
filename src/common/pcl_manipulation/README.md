# PCL Manipulation Package

## Nodes

**pcl_manipulation** : node that wraps and advertises PointCloud manipulation functions as services.

## Services

| Service        | Inputs           | Outputs  | Description  |
| ------------- |-------------| -----| ------|
| /pcl_manipulation/segment_plane      | sensor_msgs/PointCloud2 points</br>float32 distance_threshold | sensor_msgs/PointCloud2 points | Negative planar segmentation on cloud, using distance threshold to determine inliers. |
| /pcl_manipulation/remove_outliers      | sensor_msgs/PointCloud2 points</br>int32 neighbours</br>float32 stddev</br>int32 iterations      |   sensor_msgs/PointCloud2 points | Removes outliers from cloud, inliers are determined by number of neighbours and standard deviation. |
| /pcl_manipulation/downsample | sensor_msgs/PointCloud2 points</br>float32 leaf_size_x</br>float32 leaf_size_y</br>float32 leaf_size_z      |    sensor_msgs/PointCloud2 points | Performs downsampling on cloud using VoxelGrid and specified leaf sizes.|
| /pcl_manipulation/cluster | sensor_msgs/PointCloud2 points | sensor_msgs/PointCloud2[] clusters | Performs euclidian clustering on cloud. |
| /pcl_manipulation/cartesian_filter | sensor_msgs/PointCloud2 points</br>std_msgs/String axis</br>float32 lower</br>float32 upper | sensor_msgs/PointCloud2 points | Cartesian filtering on cloud using specified axis and bounds. |
| /pcl_manipulation/min_max | sensor_msgs/PointCloud2 points | float32 min_x</br>float32 min_y</br>float32 min_z</br>float32 max_x</br>float32 max_y</br>float32 max_z | Calculate min and max of cloud. |
| /pcl_manipulation/centroid | sensor_msgs/PointCloud2 points | geometry_msgs/PointStamped centroid | Compute centroid of cloud. |
| /pcl_manipulation/combine_clouds | sensor_msgs/PointCloud2[] points | sensor_msgs/PointCloud2 points | Combines clouds into single cloud. |
## References
https://pcl.readthedocs.io/projects/tutorials/en/latest/
