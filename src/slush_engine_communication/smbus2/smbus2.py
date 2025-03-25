# smbus2/smbus2.py
"""Minimal mock for smbus2 to bypass import errors on non-Pi systems.delete later"""

class SMBus:
    def __init__(self, bus=None):
        print(f"Mock SMBus: Initialized bus {bus}")

    def write_byte_data(self, addr, reg, value):
        print(f"Mock SMBus: Writing {value} to addr {addr}, reg {reg}")

    def read_byte_data(self, addr, reg):
        return 0  # Dummy value

    def close(self):
        print("Mock SMBus: Closed")
