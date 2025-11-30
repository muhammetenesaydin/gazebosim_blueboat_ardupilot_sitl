from enum import Enum, auto

class MissionState(Enum):
    IDLE = auto()
    NAVIGATING = auto()
    AVOIDING_OBSTACLE = auto()
    DOCKING = auto()
    COMPLETED = auto()

class MissionController:
    """
    Manages the high-level state of the mission.
    Follows the State Pattern.
    """
    def __init__(self):
        self.state = MissionState.IDLE
        
    def update(self, waypoint_reached, obstacle_detected, target_detected):
        if self.state == MissionState.IDLE:
            self.state = MissionState.NAVIGATING
            
        elif self.state == MissionState.NAVIGATING:
            if obstacle_detected:
                self.state = MissionState.AVOIDING_OBSTACLE
            elif target_detected:
                self.state = MissionState.DOCKING
            elif waypoint_reached:
                # Check if all waypoints done? Handled by planner usually
                pass
                
        elif self.state == MissionState.AVOIDING_OBSTACLE:
            if not obstacle_detected:
                self.state = MissionState.NAVIGATING
                
        elif self.state == MissionState.DOCKING:
            # If docked
            pass
            
        return self.state
