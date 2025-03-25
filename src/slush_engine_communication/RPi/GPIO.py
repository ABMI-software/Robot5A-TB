# RPi/GPIO.py
"""Enhanced mock for RPi.GPIO to bypass hardware errors on non-Pi systems.delete later"""

BOARD = 10
BCM = 11  # Added for setmode
OUT = 1
IN = 0
HIGH = 1
LOW = 0

def setmode(mode):
    print(f"Mock GPIO: Set mode to {mode}")

def setup(pin, mode):
    print(f"Mock GPIO: Setting pin {pin} to mode {mode}")

def output(pin, value):
    print(f"Mock GPIO: Setting pin {pin} to {value}")

def cleanup():
    print("Mock GPIO: Cleanup")
