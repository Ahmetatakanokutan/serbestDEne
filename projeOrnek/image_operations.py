import cv2
import numpy as np

class ShapeDetector:
    def __init__(self, min_area=250, min_solidity=0.9):
        self.min_area = min_area
        self.min_solidity = min_solidity

    def detect(self, frame, num_sides):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detected_shapes = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue

            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)

            if len(approx) == num_sides:
                hull = cv2.convexHull(contour)
                hull_area = cv2.contourArea(hull)
                if hull_area == 0:
                    continue
                
                solidity = float(area) / hull_area
                if solidity > self.min_solidity:
                    detected_shapes.append(contour)
        
        if not detected_shapes:
            return None
        
        largest_shape = max(detected_shapes, key=cv2.contourArea)
        
        M = cv2.moments(largest_shape)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return (cx, cy)
        return None

class TriangleDetector(ShapeDetector):
    def __init__(self):
        super().__init__()

    def process_frame(self, frame, debug=True):
        return self.detect(frame, 3)

class HexagonDetector(ShapeDetector):
    def __init__(self):
        super().__init__()

    def process_frame(self, frame, debug=True):
        return self.detect(frame, 6)
