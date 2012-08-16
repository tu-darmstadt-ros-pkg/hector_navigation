# This hold basic information about the characterists of the EvaluationGrid

# The time at which the map was loaded
time map_load_time
# Map resolution in xy plane [m/cell]
float64 resolution_xy
# Map resolution in height [m/cell]
float64 resolution_z
# min observed height [m]
float64 min_elevation
# max observed height [m]
float64 max_elevation
# Height zero value. For example 16384.
int16 zero_elevation
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin


