std_msgs/Header header

# The object identifier.
# This field can represent the unique identifier of the object
# if it has one (like an aruco code).
# If objects don't have a unique ID, this field may be used to 
# represent the type of the object (pole, wall, car) or left blank
string id

# The 3D point representing the detection
geometry_msgs/Point position

# Row-major representation of the 3x3 (x, y, z) covariance matrix
# that represents the uncertainty of the detection data
float64[9] covariance
