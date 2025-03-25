# RPi/GPIO.py
"""Enhanced mock for RPi.GPIO to bypass hardware errors on non-Pi systems. Delete later."""

BOARD = 10
BCM = 11  # Added for setmode
OUT = 1
IN = 0
HIGH = 1
LOW = 0

def setmode(mode):
    # print(f"Mock GPIO: Set mode to {mode}")
    pass

def setup(pin, mode):
    # print(f"Mock GPIO: Setting pin {pin} to mode {mode}")
    pass

def output(pin, value):
    # print(f"Mock GPIO: Setting pin {pin} to {value}")
    pass

def cleanup():
    # print("Mock GPIO: Cleanup")
    pass