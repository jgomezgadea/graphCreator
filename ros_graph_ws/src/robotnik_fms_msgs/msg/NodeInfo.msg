int32 id
robot_local_control_msgs/Pose2DStamped position
geometry_msgs/Pose2D position_offset
float32 coord_z
int32 laser_range
int32 floor
int32 robot
int32 robot_reserved
bool acoustic_signal
bool recalibration
bool used
bool charge
bool load
bool unload
bool prepick
bool preplace
bool postpick
bool postplace

bool stop
bool dooropen
bool doorclose
int32 door_id
bool entershower
bool leaveshower
string showerdoor
string showerfrom
string name

bool enter_lift
bool leave_lift
string lift_floor

bool switchmap
string switchmap_name
float32 switchmap_x
float32 switchmap_y
float32 switchmap_theta

bool move

geometry_msgs/Pose2D move_goal
geometry_msgs/Twist move_maximum_velocity

string leavemagnetic_turn_direction
float32 findmagnetic_distance
string findmagnetic_turn_direction

bool magneticload
bool magneticunload
bool findmagnetic
bool leavemagnetic

uint32 magnetic_load_unload_lane
uint32 magnetic_load_unload_cart_position
string magnetic_load_unload_turn_direction
bool   magnetic_load_unload_allow_markers
string magnetic_load_unload_cart_id

bool elevator
bool elevator_get_control
bool elevator_leave_control
bool elevator_open_door
bool elevator_close_door
uint32 elevator_id
int32 elevator_floor
int32[] zones

bool do_nothing

