import numpy as np
import cv2

class LocalMapper:
    """
    Responsible for creating a local occupancy grid from depth data.
    Follows the Single Responsibility Principle (SRP).
    """
    def __init__(self, map_size_meters=20.0, resolution=0.1):
        self.map_size_meters = map_size_meters
        self.resolution = resolution
        self.grid_size = int(map_size_meters / resolution)
        self.occupancy_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        
        # Camera parameters (approximate for ZED in Gazebo)
        self.fov_h = 1.91986 # radians (~110 deg)
        self.cx = 640
        self.cy = 360
        self.fx = 640 / np.tan(self.fov_h / 2.0)

    def update_map(self, depth_image):
        """
        Updates the local occupancy grid based on the latest depth image.
        Simple projection of depth points to 2D grid.
        """
        if depth_image is None:
            return

        # Reset grid
        self.occupancy_grid.fill(0)

        # Downsample for performance
        scale = 0.1
        small_depth = cv2.resize(depth_image, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        h, w = small_depth.shape
        
        # Thresholds
        min_dist = 0.5
        max_dist = 10.0
        
        # Iterate over pixels (simplified for Python, could be optimized with vectorization)
        # In a real scenario, use pointcloud projection
        
        # Vectorized approach:
        # Create coordinate grid
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        u = u * (1/scale)
        v = v * (1/scale)
        
        z = small_depth
        valid = (z > min_dist) & (z < max_dist)
        
        x = (u - self.cx) * z / self.fx
        # y = (v - self.cy) * z / fy  # We don't need height (Y) for 2D map, but we check if it's an obstacle
        
        # Z is forward, X is right in camera frame.
        # In robot frame (Base Link): X is forward, Y is left.
        # Camera frame usually: Z forward, X right, Y down.
        # So: Robot X = Camera Z, Robot Y = -Camera X
        
        robot_x = z[valid]
        robot_y = -x[valid]
        
        # Convert to grid coordinates
        # Grid center is (grid_size/2, grid_size/2)
        grid_x = (self.grid_size / 2) - (robot_y / self.resolution) # Robot Y is Left positive? No, Y is Left.
        # Let's define grid: X axis (rows) is Robot X (Forward), Y axis (cols) is Robot Y (Left)
        # Actually standard image: Row is Y, Col is X.
        # Let's stick to: Row index = Forward distance, Col index = Lateral distance
        
        # Map center (bottom-center of image) corresponds to robot position
        # Let's say map covers [0, 20m] forward and [-10m, 10m] lateral
        
        grid_row = self.grid_size - 1 - (robot_x / self.resolution)
        grid_col = (self.grid_size / 2) - (robot_y / self.resolution)
        
        # Filter valid indices
        valid_indices = (grid_row >= 0) & (grid_row < self.grid_size) & \
                        (grid_col >= 0) & (grid_col < self.grid_size)
        
        grid_row = grid_row[valid_indices].astype(int)
        grid_col = grid_col[valid_indices].astype(int)
        
        # Mark obstacles
        self.occupancy_grid[grid_row, grid_col] = 255 # Occupied

    def get_grid(self):
        return self.occupancy_grid
