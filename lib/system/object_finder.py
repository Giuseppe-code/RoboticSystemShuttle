#
# object_finder.py
#

import cv2
import numpy as np

class ObjectFinder:

    def __init__(self, thr="blue", min_area=80, max_area=25000):
        self.min_area = min_area
        self.max_area = max_area
        self.mode = "hsv"
        self.hsv_ranges = self._hsv_ranges(thr)
        if self.hsv_ranges is None:
            low_threshold, high_threshold = thr
            self.mode = "bgr"
            self.lower_limit = np.array(low_threshold, dtype=np.uint8)
            self.higher_limit = np.array(high_threshold, dtype=np.uint8)

    def _hsv_ranges(self, thr):
        if not isinstance(thr, str):
            return None
        color = thr.lower()
        if color == "blue":
            return [
                (np.array([95, 70, 70], dtype=np.uint8),
                 np.array([135, 255, 255], dtype=np.uint8)),
            ]
        if color == "red":
            return [
                (np.array([0, 45, 35], dtype=np.uint8),
                 np.array([12, 255, 255], dtype=np.uint8)),
                (np.array([170, 45, 35], dtype=np.uint8),
                 np.array([179, 255, 255], dtype=np.uint8)),
            ]
        raise ValueError("Unsupported object color: " + thr)

    def _binary_mask(self, img):
        if self.mode == "bgr":
            return cv2.inRange(img, self.lower_limit, self.higher_limit)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        binary_image = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower_limit, higher_limit in self.hsv_ranges:
            binary_image = cv2.bitwise_or(
                binary_image,
                cv2.inRange(hsv, lower_limit, higher_limit),
            )

        kernel = np.ones((5, 5), np.uint8)
        binary_image = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel)
        binary_image = cv2.morphologyEx(binary_image, cv2.MORPH_CLOSE, kernel)
        return binary_image

    def find(self, img):
        binary_image = self._binary_mask(img)
        contours, hierarchy = cv2.findContours(binary_image,
                                                cv2.RETR_EXTERNAL,
                                                cv2.CHAIN_APPROX_NONE)

        cx = -1
        cy = -1
        valid_contours = [
            contour for contour in contours
            if self.min_area <= cv2.contourArea(contour) <= self.max_area
        ]

        if valid_contours:
            cnt = max(valid_contours, key=cv2.contourArea)
            cv2.drawContours(img, [cnt], -1, (255, 255, 255))
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                cv2.circle(img, (cx, cy), 3, (0,0,0), -1)

        return cx, cy, binary_image
