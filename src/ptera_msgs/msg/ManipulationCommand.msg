# Mode selects which fields are active (others may be empty)
uint8 cmd
uint8 CMD_PICK=0
uint8 CMD_PLACE=1 # TODO: add fields for this
uint8 CMD_PLACE_REL=2 # TODO: add fields for this 

uint8 EXE_PICK=5

# Object metadata
string object_id
string object_type

# Pick operation fields
geometry_msgs/Point pick_obj_centroid_gl
sensor_msgs/PointCloud2 pick_obj_point_cloud_gl
