# BoundingBox.msg: bounding box (bbox) information used on fusion track node
std_msgs/Header header
uint32 label
 
geometry_msgs/Pose pose             # Centroid position and orientation ([m] and [rad])
geometry_msgs/Vector3 dimensions    # bbox dimension
geometry_msgs/Twist velocity        # Velocity components of centroid  ([m/s] and [rad/s])
float64 yaw                         # Yaw angle of bbox ([rad])
geometry_msgs/Vector3 color         # Color of bbox
float64 transparency                # Level of transparency of bbox color