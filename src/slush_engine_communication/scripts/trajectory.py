#!/usr/bin/env python3
import math
import time
import numpy as np
import Slush
from ikpy.chain import Chain
from ikpy.link import URDFLink

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

steps_per_rev = [
    32000, # R0_Yaw
    18000, # R1_Pitch
    6800,  # R2_Pitch
    3200,  # R3_Yaw
    12000  # R4_Pitch
]

joint_bounds = [(-math.pi/2, math.pi/2)] * 5

active_links_mask = [
    False, # base (fixed)
    False, # link0 (fixed)
    True,  # R0_Yaw (active)
    False, # link1 (fixed)
    True,  # R1_Pitch (active)
    False, # link2 (fixed)
    True,  # R2_Pitch (active)
    False, # link3 (fixed)
    True,  # R3_Yaw (active)
    False, # link4 (fixed)
    True,  # R4_Pitch (active)
    False  # end_effector (fixed)
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

def angle_to_steps(joint_index, angle_rad):
    angle_deg = math.degrees(angle_rad)
    return int((steps_per_rev[joint_index] / 360.0) * angle_deg)

# Initialize Motors
board = Slush.sBoard()
motors = [Slush.Motor(i) for i in range(5)]

for motor in motors:
    motor.resetDev()
    motor.setMicroSteps(16)
    motor.setMaxSpeed(100)
    motor.setAccel(20)
    motor.setDecel(20)
    motor.setCurrent(50,50,50,50)

# # Adjust currents for R1 and R2
# motors[1].setCurrent(80,90,90,90) # R1
# motors[0].setCurrent(80,90,90,90) # R2

# Motor mapping: R0→motors[4], R1→motors[1], R2→motors[0], R3→motors[3], R4→motors[2]
motor_map = {0:4, 1:1, 2:0, 3:3, 4:2}

# Direct final target pose (no intermediate waypoints)
target_x, target_y, target_z = (0.2, 0.0, 0.55)
initial_guess = [0]*len(robot_chain.links)

print(f"Moving directly to final pose: x={target_x:.3f}m, y={target_y:.3f}m, z={target_z:.3f}m")

target = target_transform(target_x, target_y, target_z)
ik_solution = robot_chain.inverse_kinematics_frame(
    target,
    initial_position=initial_guess
)
try:
    joint_angles = robot_chain.active_from_full(ik_solution)
    print("  Joint angles (rad):", joint_angles)

    for j, angle_rad in enumerate(joint_angles):
        steps = angle_to_steps(j, angle_rad)
        assigned_motor = motor_map[j]
        print(f"  -> Motor {assigned_motor} for Joint R{j}: {steps} steps (angle: {angle_rad:.4f} rad)")
        motors[assigned_motor].goTo(steps)

    while any(m.isBusy() for m in motors):
        time.sleep(0.1)

    print("Final pose reached.")


except KeyboardInterrupt:
    # If you press Ctrl+C at any time, the code jumps here
    print("Movement interrupted by user!")
    # Free all motors
    for m in motors:
        m.free()
    print("Motors freed.")


user_input = input("Press 'q' to free the motors and quit: ")
if user_input.lower() == 'q':
    for motor in motors:
        motor.free()

print("All motors are now free. End of script.")
