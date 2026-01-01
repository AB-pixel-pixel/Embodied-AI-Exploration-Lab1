#!/usr/bin/env python3
import cv2
import numpy as np
import os

def create_images():
    output_dir = os.path.join(os.path.dirname(__file__), '../demo_images')
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # 1. Simple Green Square (Matching Gazebo Green)
    img1 = np.zeros((480, 640, 3), dtype=np.uint8)
    # OpenCV uses BGR format, and Gazebo's green is (0.0, 1.0, 0.0) which is pure green in RGB.
    # In BGR, pure green is (0, 255, 0)
    cv2.rectangle(img1, (200, 150), (440, 330), (0, 255, 0), -1)  # Green in BGR
    cv2.imwrite(os.path.join(output_dir, 'green_square.png'), img1)

    # 2. Green Circle with Noise (Matching Gazebo Green)
    img2 = np.full((480, 640, 3), 255, dtype=np.uint8)  # White background
    cv2.circle(img2, (320, 240), 100, (0, 255, 0), -1)  # Green in BGR
    # Add some noise
    noise = np.random.randint(0, 50, (480, 640, 3), dtype=np.uint8)
    img2 = cv2.add(img2, noise)
    cv2.imwrite(os.path.join(output_dir, 'green_circle_noisy.png'), img2)

    # 3. Multiple Objects (Red, Green, Blue) - With Green as Gazebo Green
    img3 = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(img3, (50, 50), (200, 200), (255, 0, 0), -1)  # Blue in BGR
    cv2.rectangle(img3, (250, 50), (400, 200), (0, 255, 0), -1)  # Green in BGR (Matching Gazebo Green)
    cv2.rectangle(img3, (450, 50), (600, 200), (0, 0, 255), -1)  # Red in BGR
    cv2.imwrite(os.path.join(output_dir, 'multi_color.png'), img3)

    print(f"Generated 3 images in {output_dir}")

if __name__ == '__main__':
    create_images()
