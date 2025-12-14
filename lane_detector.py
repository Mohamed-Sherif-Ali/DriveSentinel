#!/usr/bin/env python3
"""
DriveSentinel - Lane Detection Module
Detects road lane markings and determines vehicle position relative to lane center
Uses computer vision techniques: edge detection and Hough line transforms
"""

import cv2
import numpy as np
import time
import signal
import sys

# ═════════════════════════════════════════════════════════════════════
# CONFIGURATION PARAMETERS
# ═════════════════════════════════════════════════════════════════════

WEBCAM_IDX = 0                        # Camera index to use
FRAME_WIDTH = 320                     # Video frame width in pixels
FRAME_HEIGHT = 240                    # Video frame height in pixels
LANE_CENTER_TOLERANCE = 15            # Pixel tolerance for "centered" position
IMAGE_SAVE_INTERVAL = 100             # Save debug image every N frames
POSITION_CHANGE_THRESHOLD = True      # Save image when position changes

# ═════════════════════════════════════════════════════════════════════
# SIGNAL HANDLING
# ═════════════════════════════════════════════════════════════════════

running = True  # Global flag to control main loop

def signal_handler(sig, frame):
    """
    Handles interrupt signals (Ctrl+C) for graceful shutdown
    Allows loop to exit cleanly and release resources
    """
    global running
    print("\n[!] Stopping lane detection...")
    running = False

# Register signal handler for Ctrl+C
signal.signal(signal.SIGINT, signal_handler)

# ═════════════════════════════════════════════════════════════════════
# LANE DETECTION FUNCTIONS
# ═════════════════════════════════════════════════════════════════════

def region_of_interest(img):
    """
    Applies a mask to focus on the region where lanes are expected
    Keeps only the lower half of the frame (where road is typically visible)
    
    Args:
        img: Grayscale edge-detected image
    
    Returns:
        Masked image with only region of interest visible
    """
    height, width = img.shape
    
    # Define rectangular ROI polygon covering bottom half of frame
    # Points: top-left, top-right, bottom-right, bottom-left
    polygons = np.array([
        [(0, int(0.5 * height)), (width, int(0.5 * height)),
         (width, height), (0, height)]
    ])
    
    # Create blank mask and fill polygon with white
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, polygons, 255)
    
    # Apply mask to image using bitwise AND
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def compute_lane_lines(lines):
    """
    Separates detected line segments into left and right lanes
    Filters invalid lines and averages multiple segments for each lane
    
    Args:
        lines: Array of line segments from HoughLinesP
               Each line: [[x1, y1, x2, y2]]
    
    Returns:
        Tuple of (left_lane, right_lane) as (slope, intercept) pairs
        Returns None for lanes that couldn't be detected
    """
    if lines is None:
        return None, None
    
    left_lines = []   # Will store (slope, intercept) for left lane segments
    right_lines = []  # Will store (slope, intercept) for right lane segments
    
    for line in lines:
        x1, y1, x2, y2 = line[0]  # Extract line endpoints
        
        # Skip nearly vertical lines (prevents unstable slope calculations)
        if abs(x2 - x1) < 10:
            continue
        
        try:
            # Fit a line to these two points: y = mx + b
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            intercept = parameters[1]
            
            # Filter out unreasonable slopes (too flat or too steep)
            if abs(slope) < 0.3 or abs(slope) > 3.0:
                continue
            
            # Classify line as left or right based on slope
            # Left lanes have negative slope (go up-left)
            # Right lanes have positive slope (go up-right)
            if slope < 0:
                left_lines.append((slope, intercept))
            else:
                right_lines.append((slope, intercept))
                
        except np.linalg.LinAlgError:
            # polyfit failed (shouldn't happen but handle gracefully)
            continue
    
    # Average all line segments for each lane
    left_lane = np.average(left_lines, axis=0) if left_lines else None
    right_lane = np.average(right_lines, axis=0) if right_lines else None
    
    return left_lane, right_lane

def compute_lane_midpoint(left_lane, right_lane, y):
    """
    Calculates the x-coordinate of the lane center at a given y-position
    Lane center is the midpoint between left and right lane lines
    
    Args:
        left_lane: (slope, intercept) tuple for left lane
        right_lane: (slope, intercept) tuple for right lane
        y: Y-coordinate at which to compute midpoint (typically frame bottom)
    
    Returns:
        X-coordinate of lane center, or None if calculation fails
    """
    if left_lane is None or right_lane is None:
        return None
    
    # Prevent division by zero (horizontal lines have slope ≈ 0)
    if abs(left_lane[0]) < 0.01 or abs(right_lane[0]) < 0.01:
        return None
    
    try:
        # Calculate x-coordinate where each lane crosses the given y-position
        # Rearranging y = mx + b to get x = (y - b) / m
        left_x = int((y - left_lane[1]) / left_lane[0])
        right_x = int((y - right_lane[1]) / right_lane[0])
        
        # Sanity check: lanes should be reasonable distance apart
        lane_width = abs(right_x - left_x)
        if lane_width < 30 or lane_width > 250:
            return None  # Unrealistic lane width, reject this result
        
        # Return midpoint between the two lanes
        return (left_x + right_x) // 2
        
    except (ZeroDivisionError, OverflowError):
        # Handle edge cases gracefully
        return None

# ═════════════════════════════════════════════════════════════════════
# MAIN DETECTION LOOP
# ═════════════════════════════════════════════════════════════════════

def main():
    """
    Main lane detection loop
    Continuously processes camera frames to detect lanes and determine position
    """
    # Initialize camera with specified resolution
    cap = cv2.VideoCapture(WEBCAM_IDX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    
    # Verify camera opened successfully
    if not cap.isOpened():
        print(f"⚠️ Could not open camera {WEBCAM_IDX}")
        return
    
    # Initialize state variables
    frame_id = 0                  # Frame counter
    frame_center = FRAME_WIDTH // 2   # X-coordinate of frame center
    last_position = None          # Previous lane position (for change detection)
    last_save_frame = 0           # Last frame number when image was saved
    
    print("🚀 Prototype-scale lane detection running...")
    print(f"Frame size: {FRAME_WIDTH}x{FRAME_HEIGHT}")
    print(f"Center: {frame_center}")
    print("Press Ctrl+C to stop\n")
    
    try:
        while running and cap.isOpened():
            # Capture single frame from camera
            ret, frame = cap.read()
            if not ret:
                print("⚠️ Frame capture failed")
                time.sleep(0.1)
                continue

            frame_id += 1
            
            # Process every other frame to reduce CPU load (effectively 10 FPS)
            if frame_id % 2 != 0:
                continue

            # ─────────────────────────────────────────────────────────
            # LANE DETECTION PIPELINE
            # ─────────────────────────────────────────────────────────
            
            # Step 1: Convert to grayscale (edge detection works on single channel)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Step 2: Apply Gaussian blur to reduce noise
            blur = cv2.GaussianBlur(gray, (3, 3), 0)
            
            # Step 3: Detect edges using Canny edge detector
            # Thresholds: 50 (low) and 150 (high)
            edges = cv2.Canny(blur, 50, 150)
            
            # Step 4: Apply region of interest mask (focus on bottom half)
            roi = region_of_interest(edges)
            
            # Step 5: Detect lines using Hough Line Transform
            lines = cv2.HoughLinesP(
                roi,                    # Input edge image
                rho=1,                  # Distance resolution in pixels
                theta=np.pi/180,        # Angular resolution in radians (1 degree)
                threshold=30,           # Minimum votes to consider a line
                minLineLength=20,       # Minimum line length in pixels
                maxLineGap=30           # Maximum gap between segments to connect
            )

            # ─────────────────────────────────────────────────────────
            # LANE ANALYSIS
            # ─────────────────────────────────────────────────────────
            
            lane_detected = False
            lane_position = "No lane lines"
            
            if lines is not None:
                # Lines were detected, now process them
                left_lane, right_lane = compute_lane_lines(lines)
                
                if left_lane is not None and right_lane is not None:
                    # Both lanes detected successfully
                    y_bottom = FRAME_HEIGHT  # Use bottom of frame for position calc
                    midpoint = compute_lane_midpoint(left_lane, right_lane, y_bottom)
                    
                    if midpoint is not None:
                        lane_detected = True
                        
                        # Determine vehicle position relative to lane center
                        if midpoint < frame_center - LANE_CENTER_TOLERANCE:
                            lane_position = "Left lane"
                        elif midpoint > frame_center + LANE_CENTER_TOLERANCE:
                            lane_position = "Right lane"
                        else:
                            lane_position = "Centered"
                        
                        # ─────────────────────────────────────────────
                        # DEBUG IMAGE SAVING
                        # ─────────────────────────────────────────────
                        
                        should_save = False
                        
                        # Save if position changed (e.g., crossed lanes)
                        if POSITION_CHANGE_THRESHOLD and lane_position != last_position:
                            should_save = True
                        
                        # Save periodically regardless of position
                        if frame_id - last_save_frame >= IMAGE_SAVE_INTERVAL:
                            should_save = True
                        
                        if should_save:
                            # Generate timestamped filename
                            timestamp = time.strftime("%Y%m%d-%H%M%S")
                            filename = f'prototype_lane_{timestamp}_f{frame_id}.jpg'
                            cv2.imwrite(filename, frame)
                            print(f"📸 Saved: {filename}")
                            last_save_frame = frame_id
                        
                        last_position = lane_position
                        
                    else:
                        lane_position = "Midpoint calculation failed"
                        
                elif left_lane is not None or right_lane is not None:
                    # Only one lane detected
                    lane_position = "Only one lane line detected"
                else:
                    # Lines detected but all were filtered out
                    lane_position = "Lines filtered out (invalid slopes)"
            
            # Print status for current frame
            print(f"Frame {frame_id:04d} | Detected: {lane_detected} | Position: {lane_position}")
            
            # Small delay to prevent CPU overload
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\n[!] Interrupted by user")
    except Exception as e:
        print(f"\n[ERROR] Unexpected error: {e}")
    finally:
        # Clean up resources
        cap.release()
        cv2.destroyAllWindows()
        print("[+] Lane detection ended cleanly")

# ═════════════════════════════════════════════════════════════════════
# PROGRAM ENTRY POINT
# ═════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    main()
