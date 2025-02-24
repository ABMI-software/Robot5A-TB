import pandas as pd

# Step 1: Read the original CSV file
input_file = '/home/chipmunk-151/Robot5A-TB/src/chariot_control/logs/aruco_log_face_650h.csv'  # Replace with your input file name
data = pd.read_csv(input_file)

# Step 2: Filter out rows with ArUco ID 'aruco_0'
filtered_data = data[data['ArUco ID'] != 'aruco_0']

# Step 3: Calculate the average positions for each ArUco marker
# Group by 'ArUco ID' and calculate the mean of Translation X, Y, Z
average_positions = filtered_data.groupby('ArUco ID').agg({
    'Translation X': 'mean',
    'Translation Y': 'mean',
    'Translation Z': 'mean'
}).reset_index()

# Step 4: Save the average positions to a new CSV file
output_file = '/home/chipmunk-151/Robot5A-TB/src/chariot_control/logs_processed/aruco_log_face_650h_average.csv'  # Replace with your desired output file name
average_positions.to_csv(output_file, index=False)

print("Average positions saved to:", output_file)