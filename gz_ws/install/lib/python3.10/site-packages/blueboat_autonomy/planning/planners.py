import numpy as np

class GlobalPlanner:
    """
    Manages the high-level waypoints.
    """
    def __init__(self):
        # Define waypoints based on the course (approximate meters relative to start)
        # Parkur 1 (Zigzag): (0,0) -> (20,0) -> (40,15) -> (60,-15) -> (80,0)
        # Parkur 2 (Straight): (80,0) -> (130,0)
        # Parkur 3 (Target): (140, 0)
        self.waypoints = [
            (20.0, 0.0),
            (40.0, 15.0),
            (60.0, -15.0),
            (80.0, 0.0),
            (130.0, 0.0),
            (140.0, 0.0) # Near targets
        ]
        self.current_wp_index = 0
        self.acceptance_radius = 5.0

    def get_current_waypoint(self):
        if self.current_wp_index < len(self.waypoints):
            return self.waypoints[self.current_wp_index]
        return None

    def check_waypoint_reached(self, x, y):
        if self.current_wp_index >= len(self.waypoints):
            return True
            
        target = self.waypoints[self.current_wp_index]
        dist = np.sqrt((x - target[0])**2 + (y - target[1])**2)
        
        if dist < self.acceptance_radius:
            self.current_wp_index += 1
            return True
        return False

class LocalPlanner:
    """
    Decides immediate action based on local map and global goal.
    Uses a simple reactive approach (VFH-like or Bubble Rebound).
    """
    def __init__(self):
        pass

    def compute_velocity_command(self, occupancy_grid, goal_local_coords):
        """
        Returns (linear_vel, angular_vel)
        """
        # Simple logic:
        # 1. Check if path to goal is clear in occupancy grid
        # 2. If blocked, find nearest clear direction
        
        # Grid: Rows=Forward, Cols=Lateral
        # Center Col is straight ahead
        
        grid_h, grid_w = occupancy_grid.shape
        center_col = grid_w // 2
        
        # Check a corridor in front
        corridor_width = 10 # pixels
        lookahead = 50 # pixels (~5m)
        
        front_view = occupancy_grid[grid_h-lookahead:grid_h, center_col-corridor_width:center_col+corridor_width]
        
        if np.any(front_view > 128):
            # Obstacle ahead!
            # Simple avoidance: Turn left or right depending on which side is clearer
            left_view = occupancy_grid[grid_h-lookahead:grid_h, 0:center_col]
            right_view = occupancy_grid[grid_h-lookahead:grid_h, center_col:]
            
            left_sum = np.sum(left_view)
            right_sum = np.sum(right_view)
            
            if left_sum < right_sum:
                return 0.5, 0.5 # Turn Left
            else:
                return 0.5, -0.5 # Turn Right
        else:
            # Clear ahead, move towards goal
            # Calculate heading to goal
            goal_x, goal_y = goal_local_coords # relative to robot
            heading_error = np.arctan2(goal_y, goal_x)
            
            # P-Controller for heading
            k_p = 1.0
            angular_vel = k_p * heading_error
            angular_vel = np.clip(angular_vel, -1.0, 1.0)
            
            return 1.0, angular_vel
