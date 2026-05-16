std_msgs/Header header
string frame_id                                # target/reference frame for task-space fields
builtin_interfaces/Duration timeout            # overall timeout to achieve goal

# Mode selects which fields are active (others may be empty)
uint8 mode
uint8 MODE_SETPOINT=0
uint8 MODE_VELOCITY=1
uint8 MODE_TRAJECTORY=2
uint8 MODE_WAYPOINT=3                          # single pose/pose+tol “go here”

# --- SETPOINT / WAYPOINT ---
float64[] x_ref                                # state-space setpoint (nx); empty if using pose_ref
float64[] xdot_ref                             # optional desired velocity in state space
geometry_msgs/Pose pose_ref                    # task-space setpoint (if used)
geometry_msgs/Twist twist_ref                  # optional task-space twist

# Tolerances (apply to setpoint/waypoint; leave empty if not used)
float64[] tol_abs                              # per-element absolute tol (state or pose components)
float64[] tol_rel                              # per-element relative tol

# Trajectory Types available from gpc_core 
uint8 trajectory_type
uint8 TRAJ_LINE2D=0
uint8 TRAJ_CIRCLE2D=1
# TODO: add more types here and in gpc_core 

