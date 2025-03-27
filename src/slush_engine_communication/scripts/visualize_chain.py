#!/usr/bin/env python3
import math
import numpy as np
import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.link import URDFLink
from ikpy.utils.plot import init_3d_figure

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

# Define the robot chain
# Now, links are placed along the Z-axis.
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

# Create a zero configuration for the chain (all joints = 0)
initial_configuration = [0]*len(robot_chain.links)

# Initialize a 3D figure
fig, ax = init_3d_figure()

# Plot the chain in the zero configuration
robot_chain.plot(initial_configuration, ax=ax)

# Display the figure
plt.show()
