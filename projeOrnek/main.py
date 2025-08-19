import cv2
import numpy as np
from image_operations import TriangleDetector, HexagonDetector
from coordinate_calculator import pixel_to_gps
import csv
import os
from datetime import datetime
import tkinter as tk
from tkinter import simpledialog
from math import sqrt

# Configuration
CAMERA_INDEX = 0
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
OUTPUT_CSV_FILE = 'detections.csv'
CLICKED_COORDS_CSV_FILE = 'clicked_coordinates.csv'
HFOV_DEG = 78
HOME_ALTITUDE = 72.0
YAW_DEG = 0.0
PITCH_DEG = 0.0
ROLL_DEG = 0.0
MIN_MOVE_DISTANCE = 10  # Minimum distance in pixels to log a new position

# Drone's current position (latitude, longitude, altitude)
# Replace with actual drone GPS data
drone_lat = 40.7128
drone_lon = -74.0060
drone_alt = 100.0
capture_alt = drone_alt - HOME_ALTITUDE

# Global variables
mode = "detection"  # "detection" or "click"
point_id = 1
last_triangle_pos = None
last_hexagon_pos = None

def initialize_csv(file_path, headers):
    """Initializes a CSV file with headers if it doesn't exist."""
    if not os.path.exists(file_path):
        with open(file_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(headers)

def save_detection(shape, lat, lon):
    """Saves a detected shape's coordinates to the CSV file."""
    with open(OUTPUT_CSV_FILE, 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([datetime.now(), shape, lat, lon])

def on_click(event, x, y, flags, param):
    """Handles mouse clicks to get GPS coordinates."""
    global point_id
    if event == cv2.EVENT_LBUTTONDOWN:
        lat, lon = pixel_to_gps(x, y, 0, 0, FRAME_WIDTH, FRAME_HEIGHT, YAW_DEG, PITCH_DEG, ROLL_DEG, drone_lat, drone_lon, capture_alt)
        
        root = tk.Tk()
        root.withdraw()
        class_name = simpledialog.askstring(title="Label Input", prompt="Enter class name:")
        root.destroy()

        if class_name is None or class_name.strip() == "":
            class_name = "Unknown"
        
        with open(CLICKED_COORDS_CSV_FILE, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([point_id, lat, lon, class_name])
        
        print(f"[ID {point_id}] Clicked at ({x}, {y}) => GPS: ({lat:.7f}, {lon:.7f}), Class: {class_name}")
        point_id += 1

def main():
    """Main function to run the drone vision project."""
    global mode, last_triangle_pos, last_hexagon_pos
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    triangle_detector = TriangleDetector()
    hexagon_detector = HexagonDetector()

    initialize_csv(OUTPUT_CSV_FILE, ['Timestamp', 'Shape', 'Latitude', 'Longitude'])
    initialize_csv(CLICKED_COORDS_CSV_FILE, ["ID", "Latitude", "Longitude", "Class"])

    cv2.namedWindow('Drone View')
    cv2.setMouseCallback('Drone View', on_click)

    kernel = np.ones((5, 5), np.uint8)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        if mode == "detection":
            # Detect red triangles
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_red = np.array([0, 120, 70])
            upper_red = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red, upper_red)
            lower_red = np.array([170, 120, 70])
            upper_red = np.array([180, 255, 255])
            mask2 = cv2.inRange(hsv, lower_red, upper_red)
            red_mask = mask1 + mask2
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
            red_masked_frame = cv2.bitwise_and(frame, frame, mask=red_mask)
            
            triangle_center = triangle_detector.process_frame(red_masked_frame, debug=False)
            if triangle_center is not None:
                dist = sqrt((triangle_center[0] - last_triangle_pos[0])**2 + (triangle_center[1] - last_triangle_pos[1])**2) if last_triangle_pos else MIN_MOVE_DISTANCE + 1
                if dist > MIN_MOVE_DISTANCE:
                    lat, lon = pixel_to_gps(triangle_center[0], triangle_center[1], 0, 0, FRAME_WIDTH, FRAME_HEIGHT, YAW_DEG, PITCH_DEG, ROLL_DEG, drone_lat, drone_lon, capture_alt)
                    save_detection('red_triangle', lat, lon)
                    last_triangle_pos = triangle_center
                cv2.circle(frame, triangle_center, 5, (0, 255, 0), -1)
                cv2.putText(frame, "Red Triangle", (triangle_center[0] + 10, triangle_center[1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Detect blue hexagons
            lower_blue = np.array([100, 150, 0])
            upper_blue = np.array([140, 255, 255])
            blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
            blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
            blue_masked_frame = cv2.bitwise_and(frame, frame, mask=blue_mask)

            hexagon_center = hexagon_detector.process_frame(blue_masked_frame, debug=False)
            if hexagon_center is not None:
                dist = sqrt((hexagon_center[0] - last_hexagon_pos[0])**2 + (hexagon_center[1] - last_hexagon_pos[1])**2) if last_hexagon_pos else MIN_MOVE_DISTANCE + 1
                if dist > MIN_MOVE_DISTANCE:
                    lat, lon = pixel_to_gps(hexagon_center[0], hexagon_center[1], 0, 0, FRAME_WIDTH, FRAME_HEIGHT, YAW_DEG, PITCH_DEG, ROLL_DEG, drone_lat, drone_lon, capture_alt)
                    save_detection('blue_hexagon', lat, lon)
                    last_hexagon_pos = hexagon_center
                cv2.circle(frame, hexagon_center, 5, (255, 0, 0), -1)
                cv2.putText(frame, "Blue Hexagon", (hexagon_center[0] + 10, hexagon_center[1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        cv2.putText(frame, f"Mode: {mode}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow('Drone View', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            mode = "click" if mode == "detection" else "detection"

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()