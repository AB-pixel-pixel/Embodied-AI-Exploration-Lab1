import time

print("Controller started.")
target_x = 1.0
target_y = 1.0

while True:
    print(f"Controller: I want the robot to move to ({target_x}, {target_y}). sending command 'forward'...")
    # In this isolated version, we have no way to send this command to the motor process.
    time.sleep(2)
