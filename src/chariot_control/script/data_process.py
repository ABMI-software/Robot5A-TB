import csv
import math

def quaternion_to_euler(q):
    """Convert quaternion to Euler angles (roll, pitch, yaw)."""
    x, y, z, w = q
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def analyze_data(csv_file_path):
    with open(csv_file_path, mode='r') as csvfile:
        reader = csv.DictReader(csvfile)
        previous_time = None
        previous_position = None
        results = []

        for row in reader:
            command = row['Command']
            speed = float(row['Speed'])
            initial_position = float(row['Initial Position'])
            end_position = float(row['End Position'])
            current_time = float(row['Time'])
            aruco_id = row['ArUco ID']
            translation_x = float(row['Translation X'])  # Movement along the X-axis
            translation_y = float(row['Translation Y'])
            translation_z = float(row['Translation Z'])
            rotation_x = float(row['Rotation X'])
            rotation_y = float(row['Rotation Y'])
            rotation_z = float(row['Rotation Z'])
            rotation_w = float(row['Rotation W'])

            # Calculate the actual position based on translation
            actual_position = translation_x  # Assuming movement is along the X-axis

            # Calculate precision
            precision = abs(actual_position - end_position)

            # Calculate speed based on time difference
            if previous_time is not None:
                time_diff = current_time - previous_time
                distance_traveled = actual_position - previous_position if previous_position is not None else 0
                calculated_speed = distance_traveled / time_diff if time_diff > 0 else 0
            else:
                calculated_speed = speed  # Use the commanded speed for the first entry

            # Convert quaternion to Euler angles
            euler_angles = quaternion_to_euler((rotation_x, rotation_y, rotation_z, rotation_w))

            # Store results
            results.append({
                'Command': command,
                'Speed': speed,
                'Initial Position': initial_position,
                'End Position': end_position,
                'Actual Position': actual_position,
                'Precision': precision,
                'Calculated Speed': calculated_speed,
                'Euler Angles (Roll, Pitch, Yaw)': euler_angles,
                'ArUco ID': aruco_id
            })

            # Update previous values
            previous_time = current_time
            previous_position = actual_position

        # Print results
        for result in results:
            print(result)

# Path to your CSV file
csv_file_path = '/home/chipmunk-151/Robot5A-TB/src/chariot_control/logs/aruco_log_face_650h.csv'
analyze_data(csv_file_path)