int32 task_id
int32 robot_id
int16 service_type
string service_type_name
int32 task_rlc_id
string tf_dock
bool block_node
int32 door_id
string shower_door
string shower_from


bool switchmap
string switchmap_name
float32 switchmap_x
float32 switchmap_y
float32 switchmap_theta

string leave_magnetic_turn_direction
float32 find_magnetic_distance
string find_magnetic_turn_direction

bool elevator_door_open
bool elevator_door_close
uint32 elevator_id
uint32 elevator_floor
bool elevator_get_control
bool elevator_leave_control

uint32 magnetic_pick_place_lane
uint32 magnetic_pick_place_cart_position
string magnetic_pick_place_turn_direction
bool   magnetic_pick_place_allow_markers
string magnetic_pick_place_cart_id

string lift_floor

geometry_msgs/Pose2D move_goal
geometry_msgs/Twist move_maximum_velocity

int32 floor

bool demo

bool wait_end_goto
float32 dist_end_goto

int32[] vgoals_id
robot_local_control_msgs/Pose2DStamped[] vgoals_pos 

