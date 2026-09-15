import collections
try:
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping
except AttributeError:
    pass

import cv2
import numpy as np
import threading
import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# --- CAMERA PARAMETERS ---
# Modify these based on the actual camera used (e.g., Pi Camera V2)
CAMERA_FOV_H = 62.2 # Horizontal FOV in degrees
CAMERA_FOV_V = 48.8 # Vertical FOV in degrees
CAMERA_RES_W = 640
CAMERA_RES_H = 480
TARGET_DIAMETER_M = 0.5 # Physical diameter of the target in meters

# --- PID CONSTANTS ---
# Proportional and Derivative terms for X, Y velocity
Kp_xy = 0.002
Kd_xy = 0.0005

class VideoStream:
    """Threaded Video Stream for Raspberry Pi 5"""
    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_RES_W)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_RES_H)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False
        self.lock = threading.Lock()

    def start(self):
        threading.Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            grabbed, frame = self.stream.read()
            with self.lock:
                self.grabbed = grabbed
                self.frame = frame

    def read(self):
        with self.lock:
            if self.frame is not None:
                return self.frame.copy()
            return None

    def stop(self):
        self.stopped = True
        self.stream.release()

def detect_target(frame):
    """
    Detects concentric circles using OpenCV.
    Returns:
        target_found (bool): True if target is detected.
        offset_x (int): X pixel offset from center.
        offset_y (int): Y pixel offset from center.
        estimated_alt (float): Estimated altitude based on target pixel size.
        annotated_frame (numpy.ndarray): Frame with drawings.
    """
    if frame is None:
        return False, 0, 0, 0.0, frame

    # Apply CLAHE
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l_channel, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    cl = clahe.apply(l_channel)
    limg = cv2.merge((cl,a,b))
    enhanced_frame = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)

    # Convert to HSV and Threshold (Adjust range depending on the target color. Default: Red/Yellow mix)
    hsv = cv2.cvtColor(enhanced_frame, cv2.COLOR_BGR2HSV)
    # Using a broad range for demonstration, needs tuning
    lower_bound = np.array([0, 50, 50])
    upper_bound = np.array([180, 255, 255])
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    
    # Noise removal
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours using RETR_TREE
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    target_found = False
    offset_x, offset_y = 0, 0
    estimated_alt = 0.0
    best_contour = None
    max_radius = 0

    if hierarchy is not None:
        hierarchy = hierarchy[0]
        # Look for a contour with a child (indicating concentric shape)
        for i, contour in enumerate(contours):
            # Check if this contour has a child (hierarchy[i][2] != -1)
            if hierarchy[i][2] != -1:
                # Basic shape approximation to ensure it's circular-ish
                perimeter = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
                if len(approx) > 4: # Polygon with >4 vertices is a circle candidate
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    if radius > 10 and radius > max_radius:
                        max_radius = radius
                        best_contour = contour
                        target_found = True
                        
    if target_found and best_contour is not None:
        M = cv2.moments(best_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            # Draw for visualization
            cv2.drawContours(frame, [best_contour], -1, (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
            
            # Calculate offsets
            center_x = CAMERA_RES_W // 2
            center_y = CAMERA_RES_H // 2
            offset_x = cx - center_x
            offset_y = cy - center_y
            
            # Estimate altitude
            # Pixel diameter = max_radius * 2
            pixel_diameter = max_radius * 2
            if pixel_diameter > 0:
                # focal_length_px = (CAMERA_RES_W / 2) / tan(FOV_H / 2)
                focal_length_px = (CAMERA_RES_W / 2) / math.tan(math.radians(CAMERA_FOV_H / 2))
                estimated_alt = (TARGET_DIAMETER_M * focal_length_px) / pixel_diameter
            
    return target_found, offset_x, offset_y, estimated_alt, frame

def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    APM 2.8 compatible.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    vehicle.send_mavlink(msg)
    vehicle.flush()

def generate_lawnmower_waypoints(start_loc, width, length, spacing):
    """Generates a simple lawnmower pattern relative to start location"""
    waypoints = []
    # simplified representation: list of (dNorth, dEast) offsets
    rows = int(length / spacing)
    for i in range(rows):
        dNorth = i * spacing
        dEast = width if i % 2 == 0 else 0
        waypoints.append((dNorth, dEast))
        dEast2 = 0 if i % 2 == 0 else width
        waypoints.append((dNorth, dEast2))
    return waypoints

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobalRelative object containing the latitude/longitude dNorth and dEast metres from the 
    specified `original_location`.
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    return targetlocation

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobalRelative objects.
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def main():
    # 1. Connect to Drone
    # For SITL: 'tcp:127.0.0.1:5760' or 'udp:127.0.0.1:14550'
    # For Pi to APM 2.8 via Serial: '/dev/serial0' or '/dev/ttyAMA0', baud=57600
    connection_string = 'tcp:127.0.0.1:5760'
    print(f"Connecting to vehicle on: {connection_string}")
    try:
        vehicle = connect(connection_string, wait_ready=True, baud=57600)
    except Exception as e:
        print(f"Connection failed: {e}")
        return

    # 2. Start Video Stream
    print("Starting Video Stream...")
    vs = VideoStream(src=0).start()
    time.sleep(2.0) # Warmup

    target_altitude = 10.0 # meters
    landings_completed = 0
    state = "TAKEOFF"
    
    # PID Variables
    prev_error_x = 0
    prev_error_y = 0
    
    # Lawnmower Variables
    waypoints = []
    current_wp_idx = 0
    start_loc = None

    try:
        while True:
            loop_start = time.time()
            
            # --- 1. Vision Processing ---
            frame = vs.read()
            target_found, offset_x, offset_y, est_alt, ann_frame = detect_target(frame)
            
            if ann_frame is not None:
                cv2.imshow("Drone Camera View", ann_frame)
                cv2.waitKey(1)
            
            # --- 2. State Machine & Control ---
            if state == "TAKEOFF":
                print("Basic pre-arm checks")
                while not vehicle.is_armable:
                    print(" Waiting for vehicle to initialise...")
                    time.sleep(1)

                print("Arming motors")
                vehicle.mode = VehicleMode("GUIDED")
                vehicle.armed = True

                while not vehicle.armed:
                    print(" Waiting for arming...")
                    time.sleep(1)

                print("Taking off!")
                vehicle.simple_takeoff(target_altitude)

                while True:
                    print(f" Altitude: {vehicle.location.global_relative_frame.alt}")
                    if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
                        print("Reached target altitude")
                        break
                    time.sleep(1)
                
                start_loc = vehicle.location.global_relative_frame
                waypoints = generate_lawnmower_waypoints(start_loc, width=20, length=20, spacing=5)
                current_wp_idx = 0
                state = "LAWNMOWER_SEARCH"
            
            elif state == "LAWNMOWER_SEARCH":
                if target_found:
                    print("Target detected! Switching to CENTERING.")
                    state = "CENTERING"
                    # Stop current movement
                    send_ned_velocity(vehicle, 0, 0, 0) 
                else:
                    if current_wp_idx < len(waypoints):
                        target_wp = get_location_metres(start_loc, waypoints[current_wp_idx][0], waypoints[current_wp_idx][1])
                        target_wp.alt = target_altitude
                        vehicle.simple_goto(target_wp)
                        
                        dist = get_distance_metres(vehicle.location.global_relative_frame, target_wp)
                        if dist < 1.0: # Reached waypoint
                            current_wp_idx += 1
                    else:
                        print("Search completed. Target not found. RTL.")
                        state = "RTL"
                        
            elif state == "CENTERING":
                if not target_found:
                    print("Target lost. Searching...")
                    # Basic logic: wait or go back to search
                    state = "LAWNMOWER_SEARCH"
                else:
                    # PID Control for X and Y velocity
                    # offset_x is positive if target is to the right -> drone needs to move right (positive Y velocity in NED)
                    # offset_y is positive if target is below center -> drone needs to move backward (negative X velocity in NED)
                    
                    error_y = offset_x # Image X maps to Drone Y
                    error_x = -offset_y # Image Y maps to Drone X
                    
                    vel_y = (Kp_xy * error_y) + (Kd_xy * (error_y - prev_error_y))
                    vel_x = (Kp_xy * error_x) + (Kd_xy * (error_x - prev_error_x))
                    
                    prev_error_x = error_x
                    prev_error_y = error_y
                    
                    # Limit velocities
                    vel_x = max(min(vel_x, 1.0), -1.0)
                    vel_y = max(min(vel_y, 1.0), -1.0)
                    
                    # Descend slowly if centered
                    vel_z = 0.0
                    if abs(offset_x) < 30 and abs(offset_y) < 30:
                        vel_z = 0.5 # Descend at 0.5 m/s
                        if vehicle.location.global_relative_frame.alt < 1.0:
                            print("Low altitude, switching to LANDING.")
                            state = "LANDING"
                    
                    send_ned_velocity(vehicle, vel_x, vel_y, vel_z)

            elif state == "LANDING":
                vehicle.mode = VehicleMode("LAND")
                while vehicle.armed:
                    print(f"Landing... Alt: {vehicle.location.global_relative_frame.alt}")
                    time.sleep(1)
                
                print("Landed successfully.")
                landings_completed += 1
                state = "HOLD"
                
            elif state == "HOLD":
                print("Holding on ground for 3 seconds...")
                time.sleep(3)
                if landings_completed >= 3:
                    print("Completed 3 landings. RTL.")
                    state = "RTL"
                else:
                    print("Taking off for next target.")
                    state = "TAKEOFF"
                    
            elif state == "RTL":
                vehicle.mode = VehicleMode("RTL")
                print("Returning to Launch.")
                break

            # --- 3. Rate Limiting to 5Hz ---
            # APM 2.8 is easily overloaded. Max 5 iterations per second.
            elapsed = time.time() - loop_start
            sleep_time = max(0, 0.2 - elapsed) # 1/5Hz = 0.2s
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        print("Cleaning up...")
        vs.stop()
        vehicle.close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
