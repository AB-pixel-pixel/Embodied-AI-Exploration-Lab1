#!/usr/bin/env python3
import cv2
import numpy as np
import os
import glob

def nothing(x):
    pass

def main():
    # Load images
    img_dir = os.path.join(os.path.dirname(__file__), '../demo_images')
    image_files = glob.glob(os.path.join(img_dir, '*.png'))
    
    if not image_files:
        print("No images found. Run generate_images.py first.")
        return

    current_img_idx = 0
    
    cv2.namedWindow('HSV Tuner')
    
    # Create Trackbars
    cv2.createTrackbar('H Min', 'HSV Tuner', 0, 179, nothing)
    cv2.createTrackbar('S Min', 'HSV Tuner', 120, 255, nothing)
    cv2.createTrackbar('V Min', 'HSV Tuner', 70, 255, nothing)
    cv2.createTrackbar('H Max', 'HSV Tuner', 10, 179, nothing)
    cv2.createTrackbar('S Max', 'HSV Tuner', 255, 255, nothing)
    cv2.createTrackbar('V Max', 'HSV Tuner', 255, 255, nothing)
    
    cv2.createTrackbar('Image', 'HSV Tuner', 0, len(image_files)-1, nothing)

    while True:
        # Get Trackbar positions
        h_min = cv2.getTrackbarPos('H Min', 'HSV Tuner')
        s_min = cv2.getTrackbarPos('S Min', 'HSV Tuner')
        v_min = cv2.getTrackbarPos('V Min', 'HSV Tuner')
        h_max = cv2.getTrackbarPos('H Max', 'HSV Tuner')
        s_max = cv2.getTrackbarPos('S Max', 'HSV Tuner')
        v_max = cv2.getTrackbarPos('V Max', 'HSV Tuner')
        img_idx = cv2.getTrackbarPos('Image', 'HSV Tuner')
        
        # Load current image
        img = cv2.imread(image_files[img_idx])
        if img is None: continue
        
        # Resize for better display if needed
        img = cv2.resize(img, (400, 300))
        
        # Convert to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # Create Mask
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, upper)
        
        # Feature Extraction (Contours)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Result Image
        result = img.copy()
        cv2.drawContours(result, contours, -1, (0, 255, 0), 2)
        
        # Create Visualization Stack
        # 1. Original
        # 2. Mask (Gray) -> Convert to BGR for stacking
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        # 3. Result
        
        # Stack horizontally
        stacked = np.hstack((img, mask_bgr, result))
        
        cv2.imshow('HSV Tuner', stacked)
        
        key = cv2.waitKey(1) & 0xFF
        if key == 27: # ESC
            break
            
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
