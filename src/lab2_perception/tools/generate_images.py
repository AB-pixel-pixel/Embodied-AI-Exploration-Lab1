#!/usr/bin/env python3
import cv2
import numpy as np
import os

def create_images():
    output_dir = os.path.join(os.path.dirname(__file__), '../demo_images')
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # 1. Simple Red Square
    img1 = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(img1, (200, 150), (440, 330), (0, 0, 255), -1) # Red in BGR
    cv2.imwrite(os.path.join(output_dir, 'red_square.png'), img1)

    # 2. Red Circle with Noise
    img2 = np.full((480, 640, 3), 255, dtype=np.uint8) # White background
    cv2.circle(img2, (320, 240), 100, (0, 0, 255), -1)
    # Add some noise
    noise = np.random.randint(0, 50, (480, 640, 3), dtype=np.uint8)
    img2 = cv2.add(img2, noise)
    cv2.imwrite(os.path.join(output_dir, 'red_circle_noisy.png'), img2)

    # 3. Multiple Objects (Red, Blue, Green)
    img3 = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(img3, (50, 50), (200, 200), (255, 0, 0), -1) # Blue
    cv2.rectangle(img3, (250, 50), (400, 200), (0, 255, 0), -1) # Green
    cv2.rectangle(img3, (450, 50), (600, 200), (0, 0, 255), -1) # Red
    cv2.imwrite(os.path.join(output_dir, 'multi_color.png'), img3)

    print(f"Generated 3 images in {output_dir}")

if __name__ == '__main__':
    create_images()
