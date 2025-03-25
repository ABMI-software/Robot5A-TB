# RPi/GPIO.py
"""Minimal mock for RPi.GPIO to bypass import errors on non-Pi systems. remember to delete"""

BOARD = 10
OUT = 1
IN = 0
HIGH = 1
LOW = 0

def setmode(mode):
    pass

def setup(pin, mode):
    pass

def output(pin, value):
    print(f"Mock GPIO: Setting pin {pin} to {value}")

def cleanup():
    pass
