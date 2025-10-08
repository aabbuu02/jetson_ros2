# Vertex types
uint8 NORMAL=0
uint8 PARK=1
uint8 CHARGE=2
uint8 CONTROL=3
uint8 REPORT=4

uint32 id
string name
string alias
uint8 type
bool ignore_orientation
geometry_msgs/Pose pose
Property[] properties