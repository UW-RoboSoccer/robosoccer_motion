"""
dcm motion planner.
- generate footsteps
- generate dcm trajectory
- generate com trajectory

need to test - note for ernest
"""


#!/usr/bin/env python3

import numpy as np
from scipy.interpolate import CubicSpline
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3
from robosoccer_control.msg import FootstepArray, Footstep
from std_msgs.msg import Header

class DCMConfig:
    """Configuration parameters ported from tsid_control/op3_conf.py"""
    def __init__(self):
        self.dt = 0.002  # controller time step
        self.g = 9.81    # gravity
        self.z0 = 0.4    # initial height of center of mass
        
        self.step_length = 0.1    # length of the step
        self.step_height = 0.05   # height of the step  
        self.step_width = 0.1275  # width of the step
        self.step_time = 0.7      # time to complete the step

class DCMController:
    """DCM walking controller ported from tsid_control/walk_controller.py"""
    
    def __init__(self, config):
        self.conf = config
        self.dt = config.dt
        self.time_step = 0
        self.current_step = 0
        self.depth = 3  # Number of future steps to consider
        
        # Natural frequency of the linearized inverted pendulum
        self.w_n = np.sqrt(config.z0 / config.g)
        
        # Initialize state variables
        self.x = np.zeros(3)     # [x, y, z] CoM position
        self.dx = np.zeros(3)    # [dx, dy, dz] CoM velocity
        self.e = np.zeros(3)     # [x, y, z] DCM position
        self.zmp = np.zeros(2)   # [x, y] ZMP position
        
        # Initialize trajectories
        self.footsteps = []
        self.dcm_traj = []
        self.com_traj = []

    def gen_footsteps(self, path, orientation):
        """Generate footstep positions from reference path"""
        self.current_step = 0
        footsteps = []
        dist = 0
        right_foot = True  # Start with right foot

        tangent = orientation
        normal = np.array([-tangent[1], tangent[0]])
        
        # Initial footstep at origin
        footsteps.append(np.array([
            self.conf.step_width * normal[0] * -1, 
            self.conf.step_width * normal[1] * -1, 
            0, 
            False  # is_right_foot
        ]))

        # Generate discrete footstep positions
        for i in range(len(path) - 1):
            dx = path[i + 1, 0] - path[i, 0]
            dy = path[i + 1, 1] - path[i, 1]
            dist += np.sqrt(dx**2 + dy**2)
            
            if dist >= self.conf.step_length:
                tangent = np.array([dx, dy])
                tangent /= np.linalg.norm(tangent)
                normal = np.array([-tangent[1], tangent[0]])
                
                footstep = np.zeros(4)
                footstep[0] = path[i, 0] + self.conf.step_width * normal[0] * (1 if right_foot else -1)
                footstep[1] = path[i, 1] + self.conf.step_width * normal[1] * (1 if right_foot else -1)
                footstep[2] = np.arctan2(tangent[1], tangent[0])  # Yaw angle
                footstep[3] = right_foot
                footsteps.append(footstep)
                
                dist = 0
                right_foot = not right_foot

        self.footsteps = footsteps
        return footsteps

    def gen_dcm_traj(self, depth=3):
        """Generate DCM trajectory by backward recursion"""
        dcm_endpoints = []
        
        # Start from VRP at last footstep
        vrp_pos = self.footsteps[depth - 1][:2]
        dcm_end = vrp_pos

        dcm_endpoints.append(dcm_end)

        # Backward recursion
        for i in range(depth-2, 0, -1):
            vrp_pos = self.footsteps[i + self.current_step][:2]
            dcm_i = self.back_calc_dcm(vrp_pos, dcm_end)
            dcm_end = dcm_i
            dcm_endpoints.insert(0, dcm_i)

        # Generate intermediate DCM points
        self.dcm_traj = []
        for i in range(len(dcm_endpoints)):
            dcm_end = dcm_endpoints[i]
            vrp = self.footsteps[i + self.current_step][:2]
            dcm_inter_step = []

            for j in range(int(self.conf.step_time / self.dt)):
                t = j * self.dt
                dcm_t = vrp + (dcm_end - vrp) * np.exp((t - self.conf.step_time) / self.w_n)
                dcm_inter_step.append(dcm_t)

            self.dcm_traj.append(dcm_inter_step)

    def back_calc_dcm(self, vrp_pos, dcm_end):
        """Calculate initial DCM to reach dcm_end at end of step"""
        return vrp_pos + (dcm_end - vrp_pos) * np.exp(-self.conf.step_time / self.w_n)

    def gen_com_traj(self, x0, dx0):
        """Generate CoM trajectory from DCM trajectory"""
        self.com_traj = []
        self.x = x0.copy()
        self.dx = dx0.copy()

        for step_dcm in self.dcm_traj:
            com_step = []
            for dcm_ref in step_dcm:
                # CoM follows DCM with exponential convergence
                self.e[:2] = self.x[:2] + self.dx[:2] / self.w_n
                
                # Simple proportional control for now
                com_error = dcm_ref - self.e[:2]
                self.dx[:2] += 10.0 * com_error * self.dt  # Proportional gain
                self.x[:2] += self.dx[:2] * self.dt
                
                com_step.append(self.x.copy())
            self.com_traj.append(com_step)

class MotionPlannerNode(Node):
    """ROS2 node that publishes motion plans using DCM controller"""
    
    def __init__(self):
        super().__init__('dcm_motion_planner')
        
        # Initialize DCM controller
        self.config = DCMConfig()
        self.dcm_controller = DCMController(self.config)
        
        # Publishers
        self.motion_plan_pub = self.create_publisher(
            FootstepArray, '/motion_plan', 10)
        
        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped, '/move_base_simple/goal', 
            self.goal_callback, 10)
        
        # State
        self.current_goal = None
        self.motion_plan = None
        self.plan_start_time = None
        
        # Timer for publishing current motion plan
        self.timer = self.create_timer(0.1, self.publish_motion_plan)
        
        self.get_logger().info("DCM Motion Planner Node initialized")

    def goal_callback(self, msg):
        """Receive new goal and generate motion plan"""
        self.get_logger().info(f"Received new goal: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        
        # Create simple path from current position to goal
        start_pos = np.array([0.0, 0.0])  # Assume starting at origin for now
        goal_pos = np.array([msg.pose.position.x, msg.pose.position.y])
        
        # Generate straight line path
        num_points = max(10, int(np.linalg.norm(goal_pos - start_pos) / 0.1))
        path = np.linspace(start_pos, goal_pos, num_points)
        
        # Generate footsteps
        direction = goal_pos - start_pos
        if np.linalg.norm(direction) > 0:
            orientation = direction / np.linalg.norm(direction)
        else:
            orientation = np.array([1.0, 0.0])
            
        footsteps = self.dcm_controller.gen_footsteps(path, orientation)
        
        # Generate DCM and CoM trajectories
        if len(footsteps) >= 3:
            self.dcm_controller.gen_dcm_traj(min(len(footsteps), 3))
            initial_com = np.array([0.0, 0.0, self.config.z0])
            initial_com_vel = np.zeros(3)
            self.dcm_controller.gen_com_traj(initial_com, initial_com_vel)
        
        # ROS message conversion
        self.motion_plan = self.create_footstep_array_msg(footsteps)
        self.plan_start_time = self.get_clock().now()
        
        self.get_logger().info(f"Generated motion plan with {len(footsteps)} footsteps")

    def create_footstep_array_msg(self, footsteps):
        """Convert footsteps to ROS FootstepArray message"""
        msg = FootstepArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        
        current_time = 0.0
        
        for i, footstep_data in enumerate(footsteps):
            footstep = Footstep()
            
            # Pose
            footstep.pose.position.x = float(footstep_data[0])
            footstep.pose.position.y = float(footstep_data[1])
            footstep.pose.position.z = 0.0
            
            # Orientation from yaw
            yaw = footstep_data[2]
            footstep.pose.orientation.w = np.cos(yaw/2)
            footstep.pose.orientation.z = np.sin(yaw/2)
            
            # Timing
            footstep.is_right_foot = bool(footstep_data[3])
            footstep.step_time = self.config.step_time
            footstep.start_time = current_time
            footstep.is_support_foot = True  # Simplified for now
            
            # CoM position (from trajectory if available)
            if i < len(self.dcm_controller.com_traj) and self.dcm_controller.com_traj[i]:
                com_pos = self.dcm_controller.com_traj[i][0]  # First point of step
                footstep.com_position.x = float(com_pos[0])
                footstep.com_position.y = float(com_pos[1])
                footstep.com_position.z = float(com_pos[2])
            else:
                footstep.com_position.z = self.config.z0
            
            # CoM velocity (simplified)
            footstep.com_velocity = Vector3()
            
            msg.footsteps.append(footstep)
            current_time += self.config.step_time
        
        msg.current_time = 0.0
        msg.current_step_index = 0
        
        return msg

    def publish_motion_plan(self):
        """Publish current motion plan with updated timing"""
        if self.motion_plan is None or self.plan_start_time is None:
            return
            
        # Update timing based on elapsed time
        current_time = self.get_clock().now()
        elapsed = (current_time - self.plan_start_time).nanoseconds * 1e-9
        
        self.motion_plan.current_time = elapsed
        
        # Find current step index
        for i, footstep in enumerate(self.motion_plan.footsteps):
            if elapsed >= footstep.start_time and elapsed < footstep.start_time + footstep.step_time:
                self.motion_plan.current_step_index = i
                break
        
        # Update header timestamp
        self.motion_plan.header.stamp = current_time.to_msg()
        
        # Publish
        self.motion_plan_pub.publish(self.motion_plan)

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 