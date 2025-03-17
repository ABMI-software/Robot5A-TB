import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np

# Define paths
input_csv_path = '/home/chipmunk-151/Robot5A-TB/src/chariot_control/logs/aruco_log.csv'
output_pdf_path = '/home/chipmunk-151/Robot5A-TB/src/chariot_control/logs_processed/aruco_log.pdf'

# Load the CSV file
df = pd.read_csv(input_csv_path)

# Isolate aruco_0 data
aruco_0_df = df[df['ArUco ID'] == 'aruco_0'].copy()

# Ensure time starts at 0 for better readability
aruco_0_df['Time (s)'] -= aruco_0_df['Time (s)'].min()

# Calculate errors
aruco_0_df['Error X (mm)'] = aruco_0_df['Current Position (mm)'] - aruco_0_df['Translation X (mm)']
aruco_0_df['Error Y (mm)'] = aruco_0_df['Translation Y (mm)'] - 0  # Ideal Y = 0
aruco_0_df['Error Z (mm)'] = aruco_0_df['Translation Z (mm)'] - 0  # Ideal Z = 0

# Compute statistics
stats = {
    'Error X': {
        'Mean': np.abs(aruco_0_df['Error X (mm)']).mean(),  # Absolute value for mean
        'Std Dev': aruco_0_df['Error X (mm)'].std()
    },
    'Error Y': {
        'Mean': aruco_0_df['Error Y (mm)'].mean(),
        'Std Dev': aruco_0_df['Error Y (mm)'].std()
    },
    'Error Z': {
        'Mean': aruco_0_df['Error Z (mm)'].mean(),
        'Std Dev': aruco_0_df['Error Z (mm)'].std()
    },
    'Rotation X': {
        'Mean': aruco_0_df['Rotation X'].mean(),
        'Std Dev': aruco_0_df['Rotation X'].std()
    },
    'Rotation Y': {
        'Mean': aruco_0_df['Rotation Y'].mean(),
        'Std Dev': aruco_0_df['Rotation Y'].std()
    },
    'Rotation Z': {
        'Mean': aruco_0_df['Rotation Z'].mean(),
        'Std Dev': aruco_0_df['Rotation Z'].std()
    },
    'Rotation W': {
        'Mean': aruco_0_df['Rotation W'].mean(),
        'Std Dev': aruco_0_df['Rotation W'].std()
    }
}

# Create PDF report
with PdfPages(output_pdf_path) as pdf:
    # Page 1: Position and Translation X Plot
    plt.figure(figsize=(10, 6))
    plt.plot(aruco_0_df['Time (s)'].to_numpy(), aruco_0_df['Current Position (mm)'].to_numpy(), label='Current Position (mm)', color='blue')
    plt.plot(aruco_0_df['Time (s)'].to_numpy(), aruco_0_df['Translation X (mm)'].to_numpy(), label='Translation X (mm)', color='orange')
    
    # Color-code segments based on Initial and End Positions
    commands = aruco_0_df['Command'].unique()
    for cmd in commands:
        cmd_df = aruco_0_df[aruco_0_df['Command'] == cmd]
        if cmd.startswith('P'):
            start_time = cmd_df['Time (s)'].min()
            end_time = cmd_df['Time (s)'].max()
            initial_pos = cmd_df['Initial Position (mm)'].iloc[0]
            end_pos = cmd_df['End Position (mm)'].iloc[0]
            plt.axvspan(start_time, end_time, alpha=0.2, color='green' if end_pos > initial_pos else 'red',
                        label=f'{cmd}: {initial_pos} to {end_pos} mm' if cmd == commands[0] else "")

    plt.xlabel('Time (s)')
    plt.ylabel('Position (mm)')
    plt.title('Current Position vs Translation X for ArUco_0')
    plt.legend()
    plt.grid(True)
    pdf.savefig()
    plt.close()

    # Page 2: Absolute Error X Plot
    plt.figure(figsize=(10, 6))
    plt.plot(aruco_0_df['Time (s)'].to_numpy(), np.abs(aruco_0_df['Error X (mm)'].to_numpy()), label='|Error X| (mm)', color='purple')
    plt.axhline(stats['Error X']['Mean'], color='red', linestyle='--', label=f'Mean: {stats["Error X"]["Mean"]:.2f} mm')
    plt.xlabel('Time (s)')
    plt.ylabel('Absolute Error (mm)')
    plt.title('Absolute Error between Current Position and Translation X for ArUco_0')
    plt.legend()
    plt.grid(True)
    pdf.savefig()
    plt.close()

    # Page 3: Statistics Table with Units
    plt.figure(figsize=(10, 6))
    plt.axis('off')
    table_data = [
        ['Error X (mm)', f"{stats['Error X']['Mean']:.4f}", f"{stats['Error X']['Std Dev']:.4f}"],
        ['Error Y (mm)', f"{stats['Error Y']['Mean']:.4f}", f"{stats['Error Y']['Std Dev']:.4f}"],
        ['Error Z (mm)', f"{stats['Error Z']['Mean']:.4f}", f"{stats['Error Z']['Std Dev']:.4f}"],
        ['Rotation X', f"{stats['Rotation X']['Mean']:.4f}", f"{stats['Rotation X']['Std Dev']:.4f}"],
        ['Rotation Y', f"{stats['Rotation Y']['Mean']:.4f}", f"{stats['Rotation Y']['Std Dev']:.4f}"],
        ['Rotation Z', f"{stats['Rotation Z']['Mean']:.4f}", f"{stats['Rotation Z']['Std Dev']:.4f}"],
        ['Rotation W', f"{stats['Rotation W']['Mean']:.4f}", f"{stats['Rotation W']['Std Dev']:.4f}"]
    ]
    table = plt.table(cellText=table_data,
                      colLabels=['Metric', 'Mean', 'Standard Deviation'],
                      loc='center',
                      cellLoc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.2, 1.2)
    plt.title('Statistical Analysis for ArUco_0', pad=20)
    pdf.savefig()
    plt.close()

print(f"PDF report generated: {output_pdf_path}")