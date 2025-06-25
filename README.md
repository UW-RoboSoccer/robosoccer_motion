# RoboSoccer Motion Planning

DCM-based motion planning package for walking using DCM control.

## Nodes

### `dcm_motion_planner`

Main DCM motion planning node that converts navigation goals into footstep plans.

**Subscribed Topics:**
- `/move_base_simple/goal` (geometry_msgs/PoseStamped) - Navigation target

**Published Topics:**
- `/motion_plan` (robosoccer_control/FootstepArray) - Generated footstep sequence with timing

**Relevant Parameters:**
- `step_length`: 0.1 - Length of each step (m)
- `step_height`: 0.05 - Height of swing foot (m) 
- `step_width`: 0.1275 - Width between feet (m)
- `step_time`: 0.7 - Duration of each step (s)
- `z0`: 0.4 - CoM height (m)

## Usage

### Launch Motion Planner

```bash
# Launch just the motion planner
ros2 launch motion motion_planning.launch.py

# or system launch (still working on this)
ros2 launch robosoccer_complete_system.launch.py
```

### Send Navigation Goal
eg.
```bash
# Command line
ros2 topic pub /move_base_simple/goal geometry_msgs/PoseStamped "
{
  header: {frame_id: 'world'},
  pose: {
    position: {x: 1.0, y: 0.5, z: 0.0},
    orientation: {w: 1.0}
  }
}"

# RViz: Use "2D Nav Goal" tool
```

### Monitor Motion Plan

```bash
# View generated footsteps
ros2 topic echo /motion_plan

# Check planner status
ros2 node info /dcm_motion_planner
```

## Algorithm

The DCM motion planner implements the following pipeline:

1. **Path Generation**: Creates straight-line path from current position to goal
2. **Footstep Placement**: Generates alternating left/right footsteps along path
3. **DCM Planning**: Uses backward recursion to compute DCM trajectory
4. **CoM Generation**: Calculates CoM trajectory from DCM references
5. **Message Publishing**: Converts to ROS2 FootstepArray format


## Integration

The motion planner integrates with:

- **TSID Controller** (`robosoccer_control`): Subs to `/motion_plan` topic
- **Simulation** (`my_robot_description`): Executes generated joint commands
- **Visualization**: RViz for path and footstep visualization - in progress
