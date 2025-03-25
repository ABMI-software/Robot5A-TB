# spidev/spidev.py
"""Minimal mock for spidev to bypass hardware errors on non-Pi systems.delete later"""

class SpiDev:
    def __init__(self):
        self.max_speed_hz = 0
        self.bits_per_word = 8
        self.loop = False
        self.mode = 0
        print("Mock SpiDev: Initialized")

    def open(self, bus, device):
        print(f"Mock SpiDev: Opened bus {bus}, device {device}")

    def close(self):
        print("Mock SpiDev: Closed")

    def xfer2(self, data):
        print(f"Mock SpiDev: Transferring data {data}")
        return [0] * len(data)  # Return dummy response (zeros)
