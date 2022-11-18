string[] assigned # A list of IPs of all assigned robots (Areas can have more than one robot assigned at a time)
uint32 ROBOT_POSITION=0
uint32 STAGING_POSITION=1
uint32 CHARGING_STATION=2
uint32 AREA=3
uint32 ELEVATOR_ENTRY_POSITION=26
uint32 ELEVATOR_POSITION=25
uint32 type # The resource type
uint32 path_idx # The index from the global path in which the robot gets into the position
float32 distance # The distance from the robot to the resource
geometry_msgs/Point collision_point # The collision point with the resource
geometry_msgs/Point[] resource_geometry # The resource_geometry
string[] queue # The queue for a resource. It's a list of robots ips.
string name # The name of the resource
string guid # The guid of the resource
