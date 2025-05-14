# robosoccer_motion
Plans footsteps and body movement paths

### Publishes
* `String`: `/footsteps`: Where to put next foot down
* `PoseStamped`: `/target_pose`: Robot target position

### Subscribes to
* `String`: `/behaviour_state`: What the robot should do
* `JointState`: `/joint_state`: Robot limb joint states
* `Float32`: `/imu/data`: IMU reading from robot
* `Float32`: `/imu/euler_ori`: IMU Euler Orientation from robot
* `Float32`: `/left/force`: Left foot force reading
* `Float32`: `/right/force`: Right foot force reading