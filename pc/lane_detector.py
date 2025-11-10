import cv2
import numpy as np
import time


def region_of_interest(img):
    height, width = img.shape
    # Rectangular ROI for prototype scale
    polygons = np.array(
        [
            [
                (0, int(0.5 * height)),
                (width, int(0.5 * height)),
                (width, height),
                (0, height),
            ]
        ]
    )
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def compute_lane_lines(lines):
    left_lines, right_lines = [], []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        # Skip almost vertical lines to prevent unstable slope calculations
        if abs(x2 - x1) < 10:
            continue
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        intercept = parameters[1]
        if slope < 0:
            left_lines.append((slope, intercept))
        else:
            right_lines.append((slope, intercept))
    left_lane = np.average(left_lines, axis=0) if left_lines else None
    right_lane = np.average(right_lines, axis=0) if right_lines else None
    return left_lane, right_lane


def compute_lane_midpoint(left_lane, right_lane, y):
    if left_lane is None or right_lane is None:
        return None
    left_x = int((y - left_lane[1]) / left_lane[0])
    right_x = int((y - right_lane[1]) / right_lane[0])
    return (left_x + right_x) // 2


# Initialize webcam
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

frame_id = 0
width = 320
frame_center = width // 2

print("Prototype-scale lane detection running...")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        continue

    frame_id += 1
    if frame_id % 2 != 0:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    edges = cv2.Canny(blur, 50, 150)

    roi = region_of_interest(edges)
    lines = cv2.HoughLinesP(
        roi, 1, np.pi / 180, threshold=30, minLineLength=20, maxLineGap=30
    )

    lane_detected = False
    lane_position = "No lane lines"
    if lines is not None:
        left_lane, right_lane = compute_lane_lines(lines)
        y_bottom = 240  # Bottom of frame
        midpoint = compute_lane_midpoint(left_lane, right_lane, y_bottom)
        lane_detected = True
        if midpoint:
            if midpoint < frame_center - 15:
                lane_position = "Left lane"
            elif midpoint > frame_center + 15:
                lane_position = "Right lane"
            else:
                lane_position = "Centered"
            print(f"Prototype lane position: {lane_position}")
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            cv2.imwrite(f"prototype_lane_{timestamp}.jpg", frame)
        else:
            print("Only one lane line detected.")
    else:
        lane_position = "No lane lines"

    print(
        f"Frame {frame_id} | Lane Detected: {lane_detected} | Position: {lane_position}"
    )

cap.release()
print("Lane detection ended.")
