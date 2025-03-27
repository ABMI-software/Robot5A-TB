import Slush
import time

# Initialize the SlushEngine board
board = Slush.sBoard()

# Initialize motor objects for each motor
motors = [Slush.Motor(i) for i in range(5)]

# Reset and configure each motor
for i, motor in enumerate(motors):
    motor.resetDev()                  # Reset the motor
    motor.setMicroSteps(16)           # Set microstepping to 1/16 for smoother motion
    motor.setMaxSpeed(500)            # Set a low max speed for slow motion
    motor.setAccel(30)                # Set acceleration
    motor.setDecel(30)                # Set deceleration
    motor.setCurrent(50, 50, 50, 50)  # Set motor currents (adjust as needed)

motors[1].setCurrent(70, 90, 90, 80)
# Function to move motors one by one
def move_motors_slowly():
    steps = 2000  # Number of steps to move each motor

    for i, motor in enumerate(motors):
        print(f"Moving Motor {i} forward...")
        motor.move(steps)  # Move the motor forward
        while motor.isBusy():  # Wait for the motor to finish moving
            time.sleep(0.1)
        
        print(f"Moving Motor {i} backward...")
        motor.move(-steps)  # Move the motor backward
        while motor.isBusy():  # Wait for the motor to finish moving
            time.sleep(0.1)

        print(f"Motor {i} movement complete.\n")

# Run the motor movement function
if __name__ == "__main__":
    move_motors_slowly()

    # Free all motors at the end
    for motor in motors:
        motor.free()

    print("All motors have completed their movements and are now free.")
