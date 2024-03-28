std_msgs/Header header 

# General track information
uint32 id 
string label
std_msgs/ColorRGBA color 

# 3D BBox 
geometry_msgs/Pose pose            # Relative to centroid of bounding box
geometry_msgs/Vector3 dimension
geometry_msgs/Twist velocity
geometry_msgs/Twist acceleration 

# Cluster information
sensor_msgs/PointCloud2 pointcloud