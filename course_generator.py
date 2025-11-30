import math

def generate_buoy_xml(name, x, y, model_uri, color=None):
    xml = f"""
    <include>
      <name>{name}</name>
      <pose>{x:.2f} {y:.2f} 0 0 0 0</pose>
      <uri>{model_uri}</uri>
    </include>
    """
    return xml

def generate_sausage_buoy_xml(name, x, y, color="1 0.5 0 1"):
    # Using a simple cylinder model defined inline for "sausage" buoy
    xml = f"""
    <model name="{name}">
      <pose>{x:.2f} {y:.2f} 0.2 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.15</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.15</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
    </model>
    """
    return xml

def main():
    output = ""
    
    # Parameters
    channel_width = 10.0  # Meters
    buoy_spacing = 5.0    # Meters
    
    # --- Parkur 1: Zigzag ---
    # Waypoints for the center line
    waypoints = [
        (0, 0),
        (20, 0),    # Start straight
        (40, 15),   # Turn Left
        (60, -15),  # Turn Right
        (80, 0)     # Back to center
    ]
    
    # Generate side buoys for Parkur 1
    for i in range(len(waypoints) - 1):
        p1 = waypoints[i]
        p2 = waypoints[i+1]
        
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dist = math.sqrt(dx*dx + dy*dy)
        angle = math.atan2(dy, dx)
        
        steps = int(dist / buoy_spacing)
        
        for s in range(steps):
            dist_along = s * buoy_spacing
            cx = p1[0] + math.cos(angle) * dist_along
            cy = p1[1] + math.sin(angle) * dist_along
            
            # Left Buoy (Sausage)
            lx = cx + math.cos(angle + math.pi/2) * (channel_width / 2)
            ly = cy + math.sin(angle + math.pi/2) * (channel_width / 2)
            output += generate_sausage_buoy_xml(f"p1_left_{i}_{s}", lx, ly)
            
            # Right Buoy (Sausage)
            rx = cx + math.cos(angle - math.pi/2) * (channel_width / 2)
            ry = cy + math.sin(angle - math.pi/2) * (channel_width / 2)
            output += generate_sausage_buoy_xml(f"p1_right_{i}_{s}", rx, ry)

    # Mission Points (GN) - Blue Spheres
    for i, wp in enumerate(waypoints):
        output += generate_buoy_xml(f"GN_{i+1}", wp[0], wp[1], "model://spherical_buoy") # Note: spherical_buoy is red by default, might need custom color if strict

    # --- Parkur 2: Straight with Obstacles ---
    start_x = 80
    length = 50
    end_x = start_x + length
    
    # Side buoys
    steps = int(length / buoy_spacing)
    for s in range(steps):
        x = start_x + s * buoy_spacing
        
        # Left
        output += generate_sausage_buoy_xml(f"p2_left_{s}", x, channel_width/2)
        # Right
        output += generate_sausage_buoy_xml(f"p2_right_{s}", x, -channel_width/2)
        
        # Obstacles (Yellow Spheres) - Randomly placed in channel
        if s % 2 == 0 and s > 0: # Every other step
            # Zigzag obstacles
            offset = (channel_width / 4) * (1 if (s/2) % 2 == 0 else -1)
            output += generate_buoy_xml(f"obstacle_{s}", x, offset, "model://spherical_buoy")

    # --- Parkur 3: Targets ---
    target_x = end_x + 10
    # Red Target
    output += generate_sausage_buoy_xml("target_red", target_x, 5, "1 0 0 1")
    # Green Target
    output += generate_sausage_buoy_xml("target_green", target_x, 0, "0 1 0 1")
    # Black Target
    output += generate_sausage_buoy_xml("target_black", target_x, -5, "0 0 0 1")

    # Start Zone (BB) - Visual marker?
    # Let's put a platform or just markers
    output += generate_sausage_buoy_xml("start_gate_left", 0, 6, "0 0 1 1")
    output += generate_sausage_buoy_xml("start_gate_right", 0, -6, "0 0 1 1")

    print(output)

if __name__ == "__main__":
    main()
