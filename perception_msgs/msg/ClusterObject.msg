# ClusterObject.msg: identify object as cluster obtained by the segmentation node
std_msgs/Header header

# Clusters informations
uint32 id
string label
sensor_msgs/PointCloud2 cloud  

# Bounding box (bbox) informations
BoundingBox bbox