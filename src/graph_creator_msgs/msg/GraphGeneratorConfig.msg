uint8 GRAPH_GENERATION_TYPE_GRID=0
uint8 GRAPH_GENERATION_TYPE_DXF=1

uint8 generation_type
EdgeCreationType edge_creation_type
EdgeDirectionCreationType edge_direction_type
bool ignore_vertex_orientation                      # Ignore vertex orientation
bool use_edge_count_threshold                       # Use edge count threshold to ignore orientation
uint32 edge_count_threshold                         # Edge count threshold

# For generating graph from DXF files
string dxf_file_path                                # File paths can be either relative to a package or absolute
float32 scale_x
float32 scale_y

# For generating grid graphs
geometry_msgs/Pose start_pose
uint32 rows                                         # Rows are along Y-axis
uint32 columns                                      # Columns are along X-axis
float32 row_spacing                                 # Spacing between two vertices along the Y-axis
float32 column_spacing                              # Spacing between two vertics along the X-axis
