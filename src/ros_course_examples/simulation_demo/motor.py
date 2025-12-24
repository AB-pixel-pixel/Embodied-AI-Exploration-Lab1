import time

print("Motor started.")
x = 0.0
y = 0.0

while True:
    print(f"Motor: Current position is ({x}, {y}). Waiting for commands...")
    # In this isolated version, we cannot receive commands from the controller.
    # So the robot stays at (0,0).
    time.sleep(2)
