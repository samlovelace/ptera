uint8 PICK=0
uint8 PLACE=1

uint8 operation_type

# Object metadata
string object_id
string object_type

# Pick operation fields
geometry_msgs/Point pick_obj_centroid_gl
sensor_msgs/PointCloud2 pick_obj_point_cloud_gl

# Place operation fields
geometry_msgs/Pose place_pose
