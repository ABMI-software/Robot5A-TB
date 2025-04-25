#!/usr/bin/env python3
import math
import numpy as np
from ikpy.chain import Chain
from ikpy.link import URDFLink

# ---------------------------
# Robot Dimensions (mm)
L0 = 157.0
L1 = 163.0
L2 = 210.0
L3 = 160.0
L4 = 90.0
L5 = 55.0

# Convert mm to meters
L0_m = L0 / 1000.0
L1_m = L1 / 1000.0
L2_m = L2 / 1000.0
L3_m = L3 / 1000.0
L4_m = L4 / 1000.0
L5_m = L5 / 1000.0

# Joint angle limits in radians (-90° to 90°)
joint_bounds = [(-math.pi/2, math.pi/2)] * 5

# Active links mask:
# 0: base (fixed)
# 1: link0 (fixed)
# 2: R0_Yaw (revolute)
# 3: link1 (fixed)
# 4: R1_Pitch (revolute)
# 5: link2 (fixed)
# 6: R2_Pitch (revolute)
# 7: link3 (fixed)
# 8: R3_Yaw (revolute)
# 9: link4 (fixed)
# 10: R4_Pitch (revolute)
# 11: end_effector (fixed)
active_links_mask = [
    False, # base
    False, # link0
    True,  # R0_Yaw
    False, # link1
    True,  # R1_Pitch
    False, # link2
    True,  # R2_Pitch
    False, # link3
    True,  # R3_Yaw
    False, # link4
    True,  # R4_Pitch
    False  # end_effector
]

robot_chain = Chain(
    name='robot',
    links=[
        URDFLink(name="base",      origin_translation=[0,0,0],    origin_orientation=[0,0,0], joint_type='fixed'),
        URDFLink(name="link0",     origin_translation=[0,0,L0_m], origin_orientation=[0,0,0], joint_type='fixed'),
        URDFLink(name="R0_Yaw",    origin_translation=[0,0,0],    origin_orientation=[0,0,0],
                 joint_type='revolute', rotation=[0,0,1], bounds=joint_bounds[0]),
        URDFLink(name="link1",     origin_translation=[0,0,L1_m], origin_orientation=[0,0,0], joint_type='fixed'),
        URDFLink(name="R1_Pitch",  origin_translation=[0,0,0],    origin_orientation=[0,0,0],
                 joint_type='revolute', rotation=[0,1,0], bounds=joint_bounds[1]),
        URDFLink(name="link2",     origin_translation=[0,0,L2_m], origin_orientation=[0,0,0], joint_type='fixed'),
        URDFLink(name="R2_Pitch",  origin_translation=[0,0,0],    origin_orientation=[0,0,0],
                 joint_type='revolute', rotation=[0,1,0], bounds=joint_bounds[2]),
        URDFLink(name="link3",     origin_translation=[0,0,L3_m], origin_orientation=[0,0,0], joint_type='fixed'),
        URDFLink(name="R3_Yaw",    origin_translation=[0,0,0],    origin_orientation=[0,0,0],
                 joint_type='revolute', rotation=[0,0,1], bounds=joint_bounds[3]),
        URDFLink(name="link4",     origin_translation=[0,0,L4_m], origin_orientation=[0,0,0], joint_type='fixed'),
        URDFLink(name="R4_Pitch",  origin_translation=[0,0,0],    origin_orientation=[0,0,0],
                 joint_type='revolute', rotation=[0,1,0], bounds=joint_bounds[4]),
        URDFLink(name="end_effector", origin_translation=[0,0,L5_m], origin_orientation=[0,0,0], joint_type='fixed')
    ],
    active_links_mask=active_links_mask
)

def target_transform(x, y, z):
    transform = np.eye(4)
    transform[0,3] = x
    transform[1,3] = y
    transform[2,3] = z
    return transform

# Steps per revolution for each joint:
steps_per_rev = [
    32000, # R0_Yaw
    18000, # R1_Pitch
    6800,  # R2_Pitch
    3200,  # R3_Yaw
    12000  # R4_Pitch
]

# Motor mapping (same as before)
# R0 → motors[4]
# R1 → motors[1]
# R2 → motors[0]
# R3 → motors[3]
# R4 → motors[2]
motor_map = {0:4, 1:1, 2:0, 3:3, 4:2}

def angle_to_steps(joint_index, angle_rad):
    angle_deg = math.degrees(angle_rad)
    return int((steps_per_rev[joint_index] / 360.0) * angle_deg)

# Trajectory: from home (0,0,0.835) to (0.2,0,0.55)
num_waypoints = 25
start_point = (0.0, 0.0, 0.835)
end_point   = (0.2, 0.0, 0.55)

waypoints = []
for i in range(num_waypoints+1):
    t = i/float(num_waypoints)
    x = start_point[0] + t*(end_point[0]-start_point[0])
    y = start_point[1] + t*(end_point[1]-start_point[1])
    z = start_point[2] + t*(end_point[2]-start_point[2])
    waypoints.append((x,y,z))

initial_guess = [0]*len(robot_chain.links)

for idx, (x,y,z) in enumerate(waypoints):
    print(f"Moving to waypoint {idx}/{num_waypoints}: x={x:.3f}m, y={y:.3f}m, z={z:.3f}m")
    target = target_transform(x,y,z)

    ik_solution = robot_chain.inverse_kinematics_frame(
        target,
        initial_position=initial_guess
    )

    joint_angles = robot_chain.active_from_full(ik_solution)

    # Print the joint angles and corresponding steps (no plotting)
    print("  Joint angles (rad):", joint_angles)
    for j, angle_rad in enumerate(joint_angles):
        steps = angle_to_steps(j, angle_rad)
        assigned_motor = motor_map[j]
        print(f"  -> Motor {assigned_motor} for Joint R{j}: {steps} steps (angle: {angle_rad:.4f} rad)")

    initial_guess = ik_solution
