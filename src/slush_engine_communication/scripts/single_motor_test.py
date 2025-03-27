import Slush
import time

# Initialize the SlushEngine board
board = Slush.sBoard()

# Motor 1 is struggling, increase its current
motors = [Slush.Motor(i) for i in range(5)]

# Reset and configure each motor
for i, motor in enumerate(motors):
    motor.resetDev()                  # Reset the motor
    motor.setMicroSteps(16)           # Set microstepping to 1/16 for smoother motion
    motor.setMaxSpeed(500)            # Set a low max speed for slow motion
    motor.setAccel(30)                # Set acceleration
    motor.setDecel(30)                # Set deceleration
    motor.setCurrent(50, 50, 50, 50)  # Set motor currents (adjust as needed)

R0 = motors[1]

motors[0].free()

#R0.setCurrent(80,80,80,80)
steps = 4500  # Number of steps to move each motor

print(f"R0 forward...")
R0.move(steps)  # Move the motor forward
while R0.isBusy():  # Wait for the motor to finish moving
    time.sleep(0.1)

time.sleep(5)

print(f"R0 forward...")
R0.move(-2*steps)  # Move the motor forward
while R0.isBusy():  # Wait for the motor to finish moving
    time.sleep(0.1)

time.sleep(5)

print(f"R0 forward...")
R0.move(steps)  # Move the motor forward
while R0.isBusy():  # Wait for the motor to finish moving
    time.sleep(0.1)

print(f"R0 movement complete.\n")

for motor in motors:
        motor.free()