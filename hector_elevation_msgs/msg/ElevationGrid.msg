# This represents a 2-D grid map, in which each cell represents the elevation.

Header header 

#MetaData for the map
ElevationMapMetaData info

# The map data, in row-major order, starting with (0,0).  Elevation values 
# are in the range [0,32768].
# -> 0 belongs to max negative elevation, 32767 to max positive elevation, "elevation_zero_level" coresponds to zero, -elevation_zero_level to unknown elevation
int16[] data
