#!/usr/bin/env python3
"""
DriveSentinel - Driver Drowsiness Detection System
Monitors driver alertness using facial landmark detection and controls vehicle autopilot
"""

import cv2
import mediapipe as mp
import numpy as np
import time
import sqlite3
import serial
import threading
import signal
import sys

# ═════════════════════════════════════════════════════════════════════
# CONFIGURATION PARAMETERS
# ═════════════════════════════════════════════════════════════════════

# Camera and hardware settings
WEBCAM_IDX        = 0                    # Default webcam index
ESP32_PORT        = "/dev/serial0"       # Serial port for ESP32 communication
ESP32_BAUD        = 115200               # Baud rate for serial communication

# Drowsiness detection thresholds
EAR_THRESH        = 0.25                 # Eye Aspect Ratio threshold (below = eyes closed)
EAR_CONSEC_FRAMES = 20                   # Number of consecutive frames with closed eyes before alert
ALERT_RELEASE     = 2.0                  # Seconds eyes must stay open before turning off autopilot
EAR_LOG_THRESHOLD = 0.05                 # Minimum EAR change to log (reduces database spam)

# Database configuration
DB_FILE           = "security.db"        # SQLite database file for event logging

# Thread synchronization locks
DB_LOCK           = threading.Lock()     # Protects database writes from concurrent threads
SERIAL_LOCK       = threading.Lock()     # Protects serial port writes from concurrent threads
running           = True                 # Global flag to control thread lifecycle

# ═════════════════════════════════════════════════════════════════════
# SIGNAL HANDLING
# ═════════════════════════════════════════════════════════════════════

def signal_handler(sig, frame):
    """
    Handles interrupt signals (Ctrl+C) for graceful shutdown
    Sets global running flag to False to stop all threads
    """
    global running
    print("\n[!] Shutdown signal received, cleaning up...")
    running = False
    sys.exit(0)

# Register the signal handler for SIGINT (Ctrl+C)
signal.signal(signal.SIGINT, signal_handler)

# ═════════════════════════════════════════════════════════════════════
# DATABASE SETUP
# ═════════════════════════════════════════════════════════════════════

# Connect to SQLite database (check_same_thread=False allows multi-threaded access)
conn = sqlite3.connect(DB_FILE, check_same_thread=False)
c    = conn.cursor()

# Create logs table if it doesn't exist
# Columns: timestamp (REAL), event type (TEXT), event details (TEXT)
c.execute("""
CREATE TABLE IF NOT EXISTS logs (
    ts      REAL,
    type    TEXT,
    details TEXT
)
""")
conn.commit()

def log_event(evt_type, details=""):
    """
    Logs events to both SQLite database and console output
    Thread-safe using DB_LOCK to prevent concurrent write conflicts
    
    Args:
        evt_type: Type of event (e.g., "AUTOPILOT", "ERROR", "EAR_UPDATE")
        details: Additional details about the event
    """
    ts = time.time()  # Get current Unix timestamp
    try:
        with DB_LOCK:  # Acquire lock before database operation
            c.execute("INSERT INTO logs VALUES (?, ?, ?)", (ts, evt_type, details))
            conn.commit()
        print(f"[LOG] {evt_type}: {details}")
    except sqlite3.Error as e:
        print(f"[ERROR] Database logging failed: {e}")

# ═════════════════════════════════════════════════════════════════════
# SERIAL COMMUNICATION SETUP
# ═════════════════════════════════════════════════════════════════════

ser = None  # Will hold serial connection object if successful

try:
    # Attempt to establish serial connection with ESP32
    ser = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=1)
    time.sleep(2)  # Give ESP32 time to initialize after connection
    print("✅ Connected to ESP32")
    log_event("SYSTEM", "ESP32 connected successfully")
except Exception as e:
    # If connection fails, continue without ESP32 (degraded mode)
    print(f"⚠️ Could not connect to ESP32: {e}")
    log_event("SYSTEM", f"ESP32 connection failed: {e}")
    ser = None

def send_esp(cmd: str):
    """
    Sends a command to ESP32 via serial connection
    Thread-safe using SERIAL_LOCK to prevent data corruption
    
    Args:
        cmd: Command string to send (e.g., "autopilot mode:on\n")
    
    Returns:
        bool: True if send successful, False otherwise
    """
    if not ser:
        log_event("ERROR", "Cannot send to ESP32 - not connected")
        return False
    
    try:
        with SERIAL_LOCK:  # Acquire lock before writing to serial port
            ser.write(cmd.encode())  # Convert string to bytes and send
        log_event("ESP_SEND", cmd.strip())
        return True
    except serial.SerialException as e:
        log_event("ERROR", f"Serial write failed: {e}")
        return False

# ═════════════════════════════════════════════════════════════════════
# ESP32 READER THREAD
# ═════════════════════════════════════════════════════════════════════

def esp_reader():
    """
    Background thread that continuously reads responses from ESP32
    Runs in parallel with main monitoring loop
    Uses its own database connection to avoid threading conflicts
    """
    # Create separate database connection for this thread
    local_conn = sqlite3.connect(DB_FILE, check_same_thread=False)
    local_c    = local_conn.cursor()
    
    # Keep reading while system is running and serial connection exists
    while running and ser:
        try:
            # Read one line from serial port (blocks until newline or timeout)
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            if line:  # If we received data
                ts = time.time()
                with DB_LOCK:  # Thread-safe database write
                    local_c.execute("INSERT INTO logs VALUES (?, ?, ?)",
                                    (ts, "ESP_RECV", line))
                    local_conn.commit()
                print(f"[LOG] ESP_RECV: {line}")
                
        except serial.SerialException as e:
            # Serial error occurred, log it and exit thread
            ts = time.time()
            with DB_LOCK:
                local_c.execute("INSERT INTO logs VALUES (?, ?, ?)",
                                (ts, "ESP_ERROR", str(e)))
                local_conn.commit()
            print(f"[ERROR] ESP reader error: {e}")
            break
        except Exception as e:
            print(f"[ERROR] Unexpected error in ESP reader: {e}")
            break
    
    local_conn.close()  # Clean up database connection
    print("[+] ESP reader thread terminated")

# Start the ESP reader thread only if serial connection was successful
reader_thread = None
if ser:
    reader_thread = threading.Thread(target=esp_reader, daemon=True)
    reader_thread.start()
    log_event("SYSTEM", "ESP reader thread started")

# ═════════════════════════════════════════════════════════════════════
# MEDIAPIPE FACE MESH SETUP
# ═════════════════════════════════════════════════════════════════════

# Define eye landmark indices for MediaPipe Face Mesh (468 total landmarks)
# These specific indices correspond to points around the eyes
LEFT_EYE_INDICES  = [362, 385, 387, 263, 373, 380]  # Left eye contour points
RIGHT_EYE_INDICES = [33, 160, 158, 133, 153, 144]   # Right eye contour points

# Initialize MediaPipe Face Mesh solution
mp_face_mesh = mp.solutions.face_mesh
face_mesh    = mp_face_mesh.FaceMesh(
    static_image_mode=False,           # Process video stream (not static images)
    max_num_faces=1,                   # Track only one face (the driver)
    refine_landmarks=True,             # Enable iris landmark refinement
    min_detection_confidence=0.5,      # Minimum confidence for initial detection
    min_tracking_confidence=0.5        # Minimum confidence for frame-to-frame tracking
)

def eye_aspect_ratio(lm, idxs):
    """
    Calculates Eye Aspect Ratio (EAR) from facial landmarks
    EAR measures eye openness: lower values = more closed eyes
    
    Formula: EAR = (||p2-p6|| + ||p3-p5||) / (2 * ||p1-p4||)
    where p1-p6 are the 6 eye landmark points
    
    Args:
        lm: Landmark list from MediaPipe Face Mesh
        idxs: List of 6 indices corresponding to eye landmarks
    
    Returns:
        float: Eye aspect ratio (typically 0.2-0.4 when open, <0.2 when closed)
    """
    # Extract (x,y) coordinates for the 6 eye landmarks
    pts = np.array([(lm[i].x, lm[i].y) for i in idxs])
    
    # Calculate Euclidean distances between landmark pairs
    A = np.linalg.norm(pts[1] - pts[5])  # Vertical distance 1
    B = np.linalg.norm(pts[2] - pts[4])  # Vertical distance 2
    C = np.linalg.norm(pts[0] - pts[3])  # Horizontal distance
    
    # Prevent division by zero for edge cases
    if C < 0.001:
        return 0.0
    
    # Calculate and return EAR
    return (A + B) / (2.0 * C)

# ═════════════════════════════════════════════════════════════════════
# MAIN MONITORING LOOP
# ═════════════════════════════════════════════════════════════════════

def monitor():
    """
    Main drowsiness detection loop
    Continuously captures frames, analyzes eye state, and controls autopilot
    """
    # Open camera connection
    cap = cv2.VideoCapture(WEBCAM_IDX)
    
    # Verify camera opened successfully
    if not cap.isOpened():
        print(f"⚠️ Could not open camera {WEBCAM_IDX}")
        log_event("ERROR", f"Camera {WEBCAM_IDX} failed to open")
        cap.release()
        return

    print("🚀 Driver monitoring started")
    log_event("SYSTEM", "Monitoring started")
    
    # State variables for drowsiness detection
    ear_counter  = 0        # Counts consecutive frames with eyes closed
    in_autopilot = False    # Tracks whether autopilot is currently engaged
    awake_start  = None     # Timestamp when driver's eyes reopened
    last_ear     = 0.0      # Previous EAR value (for reducing log spam)
    no_face_counter = 0     # Counts consecutive frames with no face detected
    
    try:
        while running:
            # Capture a single frame from the camera
            ret, frame = cap.read()
            if not ret:
                log_event("ERROR", "Frame capture failed")
                time.sleep(0.1)
                continue

            # Convert frame from BGR (OpenCV default) to RGB (MediaPipe requirement)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Process frame through MediaPipe Face Mesh
            results = face_mesh.process(rgb)
            
            if results.multi_face_landmarks:
                # Face detected successfully
                lm = results.multi_face_landmarks[0].landmark  # Get first (only) face
                
                # Calculate EAR for both eyes and average them
                ear = (eye_aspect_ratio(lm, LEFT_EYE_INDICES) +
                       eye_aspect_ratio(lm, RIGHT_EYE_INDICES)) / 2.0
                
                no_face_counter = 0  # Reset no-face counter since we found a face
                
                # Log EAR only when it changes significantly (reduces database spam)
                if abs(ear - last_ear) > EAR_LOG_THRESHOLD:
                    log_event("EAR_UPDATE", f"{ear:.3f}")
                    last_ear = ear
                
            else:
                # No face detected in this frame
                no_face_counter += 1
                
                # Only treat as drowsy if face has been missing for extended period
                # This prevents false positives when driver briefly looks away
                if no_face_counter > 30:  # 30 frames = ~1.5 seconds at 20 FPS
                    log_event("WARNING", "Driver not detected for extended period")
                    ear = 0.0  # Treat as eyes closed
                else:
                    # Face only briefly missing, skip this frame
                    time.sleep(0.05)
                    continue

            # ─────────────────────────────────────────────────────────
            # DROWSINESS DETECTION LOGIC
            # ─────────────────────────────────────────────────────────
            
            if ear < EAR_THRESH:
                # Eyes are closed (or mostly closed)
                ear_counter += 1
                awake_start = None  # Reset awake timer
                
                # Check if eyes have been closed long enough to trigger autopilot
                if ear_counter >= EAR_CONSEC_FRAMES and not in_autopilot:
                    in_autopilot = True
                    
                    # Send autopilot ON command to ESP32
                    if send_esp("autopilot mode:on\n"):
                        log_event("AUTOPILOT", "Mode ON sent")
                    else:
                        log_event("ERROR", "Failed to enable autopilot")
                        in_autopilot = False  # Revert state if send failed

            else:
                # Eyes are open
                ear_counter = 0  # Reset closed-eye counter
                
                if in_autopilot:
                    # Autopilot is ON, check if driver has been awake long enough
                    if awake_start is None:
                        # First frame with eyes open, start timer
                        awake_start = time.time()
                        log_event("ALERT", "Driver appears awake, monitoring...")
                        
                    elif time.time() - awake_start >= ALERT_RELEASE:
                        # Driver has been awake for required duration, turn off autopilot
                        if send_esp("autopilot mode:off\n"):
                            log_event("AUTOPILOT", "Mode OFF sent")
                        else:
                            log_event("ERROR", "Failed to disable autopilot")
                        in_autopilot = False
                        awake_start  = None

            # Control loop frequency: 50ms delay = 20 FPS
            time.sleep(0.05)
            
    except Exception as e:
        log_event("ERROR", f"Monitor loop crashed: {e}")
        print(f"[ERROR] Fatal error in monitor loop: {e}")
    finally:
        # Clean up camera resources
        cap.release()
        print("[+] Camera released")

# ═════════════════════════════════════════════════════════════════════
# PROGRAM ENTRY POINT
# ═════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    try:
        # Start the main monitoring loop
        monitor()
        
    except KeyboardInterrupt:
        print("\n[!] Exiting monitoring")
        
    finally:
        # Cleanup sequence
        running = False  # Signal all threads to stop
        
        # Wait for ESP reader thread to finish (max 2 seconds)
        if reader_thread and reader_thread.is_alive():
            reader_thread.join(timeout=2.0)
        
        # Close all resources
        conn.close()           # Close database connection
        face_mesh.close()      # Release MediaPipe resources
        
        if ser:
            ser.close()        # Close serial connection
        
        print("[+] All resources cleaned up")
        log_event("SYSTEM", "Shutdown complete")
