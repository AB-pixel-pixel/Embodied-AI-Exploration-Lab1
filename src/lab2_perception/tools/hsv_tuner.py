#!/usr/bin/env python3
import cv2
import numpy as np
import os

def nothing(x):
    pass

def main():
    img_dir = os.path.join(os.path.dirname(__file__), '../demo_images')
    image_files = [
        'green_circle_noisy.png',
        'green_square.png',
        'multi_color.png'
    ]
    image_files = [os.path.join(img_dir, f) for f in image_files]

    cv2.namedWindow('HSV Tuner')

    # HSV 滑条
    cv2.createTrackbar('H Min', 'HSV Tuner', 35, 179, nothing)
    cv2.createTrackbar('H Max', 'HSV Tuner', 85, 179, nothing)

    cv2.createTrackbar('S Min', 'HSV Tuner', 80, 255, nothing)
    cv2.createTrackbar('S Max', 'HSV Tuner', 255, 255, nothing)

    cv2.createTrackbar('V Min', 'HSV Tuner', 80, 255, nothing)
    cv2.createTrackbar('V Max', 'HSV Tuner', 255, 255, nothing)

    cv2.createTrackbar('Min Area', 'HSV Tuner', 300, 50000, nothing)

    while True:
        # 读取参数
        h_min = cv2.getTrackbarPos('H Min', 'HSV Tuner')
        s_min = cv2.getTrackbarPos('S Min', 'HSV Tuner')
        v_min = cv2.getTrackbarPos('V Min', 'HSV Tuner')
        h_max = cv2.getTrackbarPos('H Max', 'HSV Tuner')
        s_max = cv2.getTrackbarPos('S Max', 'HSV Tuner')
        v_max = cv2.getTrackbarPos('V Max', 'HSV Tuner')
        min_area = cv2.getTrackbarPos('Min Area', 'HSV Tuner')

        results = []

        for img_path in image_files:
            img = cv2.imread(img_path)
            if img is None:
                continue

            # === 1️⃣ 图片压缩到 1/2 ===
            img = cv2.resize(img, None, fx=0.3, fy=0.3)

            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            lower = np.array([h_min, s_min, v_min])
            upper = np.array([h_max, s_max, v_max])
            mask = cv2.inRange(hsv, lower, upper)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            result = img.copy()

            # === 2️⃣ 变色目标框 ===
            for i, cnt in enumerate(contours):
                if cv2.contourArea(cnt) > min_area:
                    x, y, w, h = cv2.boundingRect(cnt)

                    # 根据目标编号生成颜色
                    color = cv2.cvtColor(
                        np.uint8([[[i * 40 % 180, 255, 255]]]),
                        cv2.COLOR_HSV2BGR
                    )[0][0].tolist()

                    cv2.rectangle(
                        result,
                        (x, y),
                        (x + w, y + h),
                        color,
                        2
                    )

            # 图片名
            name = os.path.basename(img_path)
            cv2.putText(
                result,
                name,
                (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2
            )

            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            triple = np.hstack((img, mask_bgr, result))
            results.append(triple)

        if results:
            stacked_all = np.vstack(results)
            cv2.imshow('HSV Tuner', stacked_all)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
