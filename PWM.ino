/*
 * ═══════════════════════════════════════════════════════════════════
 * DriveSentinel - ESP32 Motor Control & Safety System
 * ═══════════════════════════════════════════════════════════════════
 * 
 * Manages vehicle motor control, obstacle detection, and safety logic
 * Receives autopilot commands from Raspberry Pi via serial communication
 * Implements state machine for autonomous lane switching and emergency stops
 */

#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════
// PIN DEFINITIONS - ULTRASONIC SENSORS
// ═══════════════════════════════════════════════════════════════════
// Three HC-SR04 ultrasonic sensors for obstacle detection

#define UltraTriggerFront 2    // Front sensor trigger pin (sends ultrasonic pulse)
#define UltraEchoFront    17   // Front sensor echo pin (receives pulse reflection)
#define UltraTriggerRight 18   // Right sensor trigger pin (lane boundary detection)
#define UltraEchoRight    34   // Right sensor echo pin
#define UltraTriggerBack  21   // Back sensor trigger pin (rear collision avoidance)
#define UltraEchoBack     5    // Back sensor echo pin

// ═══════════════════════════════════════════════════════════════════
// PIN DEFINITIONS - MOTOR DRIVER 2 (REAR WHEELS)
// ═══════════════════════════════════════════════════════════════════
// Controls rear left and rear right motors via L298N motor driver

#define Motor2EnA 4            // Rear Left PWM speed control (0-255)
#define Motor2EnB 12           // Rear Right PWM speed control (0-255)
#define Motor2In1 22           // Rear Left direction control pin 1
#define Motor2In2 15           // Rear Left direction control pin 2
#define Motor2In3 23           // Rear Right direction control pin 1
#define Motor2In4 16           // Rear Right direction control pin 2

// ═══════════════════════════════════════════════════════════════════
// PIN DEFINITIONS - MOTOR DRIVER 1 (FRONT WHEELS)
// ═══════════════════════════════════════════════════════════════════
// Controls front left and front right motors via L298N motor driver

#define Motor1EnA 32           // Front Left PWM speed control
#define Motor1EnB 33           // Front Right PWM speed control
#define Motor1In1 25           // Front Left direction control pin 1
#define Motor1In2 26           // Front Left direction control pin 2
#define Motor1In3 27           // Front Right direction control pin 1
#define Motor1In4 14           // Front Right direction control pin 2

// ═══════════════════════════════════════════════════════════════════
// PIN DEFINITIONS - OTHER PERIPHERALS
// ═══════════════════════════════════════════════════════════════════

#define BUZZER_PIN 13          // Piezo buzzer for emergency alerts

// ═══════════════════════════════════════════════════════════════════
// PWM CHANNEL ASSIGNMENTS
// ═══════════════════════════════════════════════════════════════════
// ESP32 has 16 independent PWM channels, we use 4 for motor control

#define PWM_CHANNEL_RL 0       // Rear Left motor PWM channel
#define PWM_CHANNEL_RR 1       // Rear Right motor PWM channel
#define PWM_CHANNEL_FL 2       // Front Left motor PWM channel
#define PWM_CHANNEL_FR 3       // Front Right motor PWM channel

// ═══════════════════════════════════════════════════════════════════
// GLOBAL STATE VARIABLES
// ═══════════════════════════════════════════════════════════════════

bool autopilotEnabled = false;        // Autopilot mode flag (controlled by Raspberry Pi)
String rxBuf = "";                    // Serial receive buffer for incoming commands
unsigned long lastLoopTime = 0;       // Timestamp of last main loop execution
const int loopInterval = 500;         // Main loop runs every 500ms (2 Hz)

// ═══════════════════════════════════════════════════════════════════
// VEHICLE PARAMETERS
// ═══════════════════════════════════════════════════════════════════
// Physical dimensions for prototype vehicle and test track

const int LANE_WIDTH = 30;            // Lane width in centimeters
const int CAR_WIDTH = 22;             // Vehicle width in centimeters
const int CAR_LENGTH = 35;            // Vehicle length in centimeters
const int ROAD_LENGTH = 300;          // Total road length (2 meters prototype track)
int currentLane = 1;                  // Current lane position (1 = left, 2 = right)

// ═══════════════════════════════════════════════════════════════════
// STATE MACHINE DEFINITION
// ═══════════════════════════════════════════════════════════════════
// Five states control autonomous driving behavior

enum SystemState { 
  STATE_DRIVING,          // Normal cruising at constant speed
  STATE_CHECKING_LANE,    // Verifying lane clearance before switching
  STATE_SWITCHING_LANE,   // Executing lane change maneuver
  STATE_SAFE_STOP,        // Stopping when rear is clear
  STATE_EMERGENCY_STOP    // Immediate stop for obstacle avoidance
};

SystemState systemState = STATE_DRIVING;  // Initial state
unsigned long stateStartTime = 0;         // Timestamp when current state began

// ═══════════════════════════════════════════════════════════════════
// TIMING CONSTANTS (milliseconds)
// ═══════════════════════════════════════════════════════════════════

const int LANE_CHECK_TIME = 1000;     // Duration to verify lane clearance (1 second)
const int LANE_SWITCH_TIME = 1500;    // Duration of lane switching maneuver (1.5 seconds)
const int EMERGENCY_TIMEOUT = 10000;  // Max time in emergency stop before forcing manual mode
const int SLOW_SPEED = 80;            // PWM value for slow speed (31% throttle)
const int NORMAL_SPEED = 110;         // PWM value for normal speed (43% throttle)

// ═══════════════════════════════════════════════════════════════════
// SAFETY THRESHOLDS (centimeters)
// ═══════════════════════════════════════════════════════════════════
// Distance thresholds for emergency stops and lane switching decisions

const int EMERGENCY_FRONT = 25;       // Minimum safe distance ahead
const int EMERGENCY_BACK = 10;        // Minimum safe distance behind
const int EMERGENCY_SIDE = 15;        // Minimum safe distance to side
const int LANE_CLEARANCE = 20;        // Required clearance to switch lanes
const int REAR_CLEARANCE = 20;        // Required rear clearance before stopping

// ═══════════════════════════════════════════════════════════════════
// SENSOR VALIDATION PARAMETERS
// ═══════════════════════════════════════════════════════════════════

const int MAX_SENSOR_DISTANCE = 400;  // Maximum valid sensor reading (cm)
const int MIN_SENSOR_DISTANCE = 2;    // Minimum valid sensor reading (cm)

// ═══════════════════════════════════════════════════════════════════
// FUNCTION DECLARATIONS
// ═══════════════════════════════════════════════════════════════════

long getDistance(int trigPin, int echoPin);  // Read ultrasonic sensor
void runAllMotors(int pwmVal);               // Set all motors to same speed
void stopAllMotors();                        // Stop all motors immediately
void gentleTurnRight(int speed);             // Execute differential turn
void handleCommand(const String &cmd);       // Process serial commands
bool isSensorValid(long distance);           // Validate sensor reading

// ═══════════════════════════════════════════════════════════════════
// SETUP - RUNS ONCE AT POWER-ON
// ═══════════════════════════════════════════════════════════════════

void setup() {
  // Initialize serial communication with Raspberry Pi
  Serial.begin(115200);
  delay(1000);  // Allow serial connection to stabilize
  
  Serial.println("\n═══════════════════════════════════════════");
  Serial.println("    DRIVESSENTINEL SYSTEM INITIALIZATION    ");
  Serial.println("═══════════════════════════════════════════");

  // ─────────────────────────────────────────────────────────────────
  // Configure ultrasonic sensor pins
  // ─────────────────────────────────────────────────────────────────
  pinMode(UltraTriggerFront, OUTPUT);   // Trigger pins send ultrasonic pulse
  pinMode(UltraEchoFront, INPUT);       // Echo pins receive reflected pulse
  pinMode(UltraTriggerRight, OUTPUT);
  pinMode(UltraEchoRight, INPUT);
  pinMode(UltraTriggerBack, OUTPUT);
  pinMode(UltraEchoBack, INPUT);
  Serial.println("✓ Ultrasonic sensors initialized");

  // ─────────────────────────────────────────────────────────────────
  // Configure motor control pins
  // ─────────────────────────────────────────────────────────────────
  // Direction control pins (digital HIGH/LOW)
  pinMode(Motor2In1, OUTPUT); pinMode(Motor2In2, OUTPUT);
  pinMode(Motor2In3, OUTPUT); pinMode(Motor2In4, OUTPUT);
  pinMode(Motor1In1, OUTPUT); pinMode(Motor1In2, OUTPUT);
  pinMode(Motor1In3, OUTPUT); pinMode(Motor1In4, OUTPUT);
  Serial.println("✓ Motor control pins initialized");

  // ─────────────────────────────────────────────────────────────────
  // Configure buzzer pin
  // ─────────────────────────────────────────────────────────────────
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);  // Start with buzzer off
  Serial.println("✓ Buzzer initialized");

  // ─────────────────────────────────────────────────────────────────
  // Configure PWM channels for motor speed control
  // ─────────────────────────────────────────────────────────────────
  // Parameters: channel, frequency (Hz), resolution (bits)
  // 5000 Hz is above human hearing range (inaudible operation)
  // 8-bit resolution provides 0-255 duty cycle values
  ledcSetup(PWM_CHANNEL_RL, 5000, 8);
  ledcSetup(PWM_CHANNEL_RR, 5000, 8);
  ledcSetup(PWM_CHANNEL_FL, 5000, 8);
  ledcSetup(PWM_CHANNEL_FR, 5000, 8);
  
  // Attach PWM channels to physical pins
  ledcAttachPin(Motor2EnA, PWM_CHANNEL_RL);
  ledcAttachPin(Motor2EnB, PWM_CHANNEL_RR);
  ledcAttachPin(Motor1EnA, PWM_CHANNEL_FL);
  ledcAttachPin(Motor1EnB, PWM_CHANNEL_FR);
  Serial.println("✓ PWM channels configured (5 kHz, 8-bit)");

  // ─────────────────────────────────────────────────────────────────
  // Safety: Ensure motors start in stopped state
  // ─────────────────────────────────────────────────────────────────
  stopAllMotors();
  
  Serial.println("\n═══════════════════════════════════════════");
  Serial.println("         INITIALIZATION COMPLETE           ");
  Serial.println("═══════════════════════════════════════════");
  Serial.println("Status: READY");
  Serial.println("Autopilot: DISABLED");
  Serial.println("Initial lane: LEFT");
  Serial.println("Waiting for commands...\n");
}

// ═══════════════════════════════════════════════════════════════════
// SERIAL COMMAND HANDLER
// ═══════════════════════════════════════════════════════════════════

void handleCommand(const String &cmd) {
  /*
   * Processes commands received from Raspberry Pi via serial
   * 
   * Valid commands:
   *   - "autopilot mode:on"  : Enable autonomous driving
   *   - "autopilot mode:off" : Disable autonomous driving (manual mode)
   * 
   * Performs input validation to prevent malformed commands
   */
  
  // Create a clean copy and remove whitespace
  String cleanCmd = cmd;
  cleanCmd.trim();
  
  // Reject empty commands
  if (cleanCmd.length() == 0) {
    return;
  }
  
  // Reject excessively long commands (potential corruption)
  if (cleanCmd.length() > 50) {
    Serial.println("ERROR:Command too long");
    return;
  }
  
  // ───────────────────────────────────────────────────────────────
  // Process AUTOPILOT ON command
  // ───────────────────────────────────────────────────────────────
  if (cleanCmd == "autopilot mode:on") {
    if (!autopilotEnabled) {
      autopilotEnabled = true;
      systemState = STATE_DRIVING;          // Reset to driving state
      stateStartTime = millis();            // Record state start time
      Serial.println("ACK:autopilot enabled");
    } else {
      Serial.println("INFO:autopilot already enabled");
    }
  }
  
  // ───────────────────────────────────────────────────────────────
  // Process AUTOPILOT OFF command
  // ───────────────────────────────────────────────────────────────
  else if (cleanCmd == "autopilot mode:off") {
    if (autopilotEnabled) {
      autopilotEnabled = false;
      stopAllMotors();                      // Immediately stop motors
      digitalWrite(BUZZER_PIN, LOW);        // Turn off emergency buzzer
      Serial.println("ACK:autopilot disabled");
    } else {
      Serial.println("INFO:autopilot already disabled");
    }
  }
  
  // ───────────────────────────────────────────────────────────────
  // Unknown command received
  // ───────────────────────────────────────────────────────────────
  else {
    Serial.print("ERROR:Unknown command: ");
    Serial.println(cleanCmd);
  }
}

// ═══════════════════════════════════════════════════════════════════
// SENSOR VALIDATION
// ═══════════════════════════════════════════════════════════════════

bool isSensorValid(long distance) {
  /*
   * Validates ultrasonic sensor reading
   * Detects sensor failures (timeout, out of range)
   * 
   * Returns: true if reading is within valid range, false otherwise
   */
  return (distance >= MIN_SENSOR_DISTANCE && distance <= MAX_SENSOR_DISTANCE);
}

// ═══════════════════════════════════════════════════════════════════
// MAIN LOOP - RUNS CONTINUOUSLY
// ═══════════════════════════════════════════════════════════════════

void loop() {
  // ───────────────────────────────────────────────────────────────
  // SERIAL COMMAND PROCESSING
  // ───────────────────────────────────────────────────────────────
  // Read incoming bytes from Raspberry Pi and parse commands
  
  while (Serial.available()) {
    char c = Serial.read();
    
    // Command complete when newline received
    if (c == '\n' || c == '\r') {
      if (rxBuf.length() > 0) {
        handleCommand(rxBuf);  // Process complete command
        rxBuf = "";            // Clear buffer for next command
      }
    } else {
      rxBuf += c;  // Accumulate characters
    }
  }

  // ───────────────────────────────────────────────────────────────
  // LOOP RATE LIMITING
  // ───────────────────────────────────────────────────────────────
  // Main control loop runs at fixed 2 Hz (every 500ms)
  // Prevents overwhelming sensor readings and motor updates
  
  if (millis() - lastLoopTime < loopInterval) {
    return;  // Not enough time elapsed, skip this iteration
  }
  lastLoopTime = millis();

  // ───────────────────────────────────────────────────────────────
  // AUTOPILOT DISABLED - MANUAL MODE
  // ───────────────────────────────────────────────────────────────
  // When autopilot is off, ensure all systems are stopped
  
  if (!autopilotEnabled) {
    stopAllMotors();
    digitalWrite(BUZZER_PIN, LOW);
    return;  // Skip autonomous control logic
  }

  // ───────────────────────────────────────────────────────────────
  // SENSOR READING
  // ───────────────────────────────────────────────────────────────
  // Poll all three ultrasonic sensors for obstacle detection
  
  long dFront = getDistance(UltraTriggerFront, UltraEchoFront);
  long dRight = getDistance(UltraTriggerRight, UltraEchoRight);
  long dBack  = getDistance(UltraTriggerBack,  UltraEchoBack);
  
  // Validate sensor readings (detect failures)
  bool frontValid = isSensorValid(dFront);
  bool rightValid = isSensorValid(dRight);
  bool backValid  = isSensorValid(dBack);
  
  // If sensor fails, assume safe distance (fail-safe design)
  if (!frontValid) dFront = MAX_SENSOR_DISTANCE;
  if (!rightValid) dRight = MAX_SENSOR_DISTANCE;
  if (!backValid)  dBack  = MAX_SENSOR_DISTANCE;
  
  // Log sensor readings with validity indicators
  Serial.print("Sensors (cm): F=");
  Serial.print(dFront);
  Serial.print(frontValid ? "" : "!");  // "!" indicates invalid reading
  Serial.print(", R=");
  Serial.print(dRight);
  Serial.print(rightValid ? "" : "!");
  Serial.print(", B=");
  Serial.print(dBack);
  Serial.println(backValid ? "" : "!");

  // ───────────────────────────────────────────────────────────────
  // EMERGENCY DETECTION
  // ───────────────────────────────────────────────────────────────
  // Check if any sensor reads dangerous proximity
  
  bool immediateDanger = (dFront < EMERGENCY_FRONT) ||
                         (dBack  < EMERGENCY_BACK)  ||
                         (dRight < EMERGENCY_SIDE);
                         
  // Trigger emergency stop if danger detected and not already in emergency state
  if (immediateDanger && systemState != STATE_EMERGENCY_STOP) {
    systemState = STATE_EMERGENCY_STOP;
    stateStartTime = millis();
    Serial.println("!!! EMERGENCY STOP TRIGGERED !!!");
  }

  // ═══════════════════════════════════════════════════════════════
  // STATE MACHINE EXECUTION
  // ═══════════════════════════════════════════════════════════════
  
  switch (systemState) {
    
    // ─────────────────────────────────────────────────────────────
    case STATE_DRIVING:
    // ─────────────────────────────────────────────────────────────
    /*
     * Normal cruising state
     * Vehicle maintains constant speed and monitors for lane switch opportunity
     */
      runAllMotors(NORMAL_SPEED);
      
      // Check if right lane is clear for switching
      if (currentLane == 1 && dRight > LANE_CLEARANCE) {
        systemState = STATE_CHECKING_LANE;
        stateStartTime = millis();
        Serial.println("→ STATE_CHECKING_LANE");
      }
      break;

    // ─────────────────────────────────────────────────────────────
    case STATE_CHECKING_LANE:
    // ─────────────────────────────────────────────────────────────
    /*
     * Verification state before committing to lane change
     * Slows down and waits 1 second to confirm lane remains clear
     * Prevents false positives from brief gaps
     */
      runAllMotors(SLOW_SPEED);
      
      // Confirm lane still clear after verification period
      if (dRight > LANE_CLEARANCE && millis() - stateStartTime > LANE_CHECK_TIME) {
        systemState = STATE_SWITCHING_LANE;
        stateStartTime = millis();
        Serial.println("→ STATE_SWITCHING_LANE (confirmed clear)");
      }
      // Abort if lane becomes blocked during check
      else if (dRight <= LANE_CLEARANCE) {
        systemState = STATE_DRIVING;
        Serial.println("→ STATE_DRIVING (lane blocked, abort)");
      }
      break;

    // ─────────────────────────────────────────────────────────────
    case STATE_SWITCHING_LANE:
    // ─────────────────────────────────────────────────────────────
    /*
     * Execute lane change maneuver
     * Uses differential steering (left motors faster than right)
     * Creates smooth arc over 1.5 seconds
     */
      gentleTurnRight(SLOW_SPEED);
      
      // Complete lane switch after maneuver duration
      if (millis() - stateStartTime > LANE_SWITCH_TIME) {
        currentLane = 2;                    // Update lane position
        systemState = STATE_SAFE_STOP;
        stateStartTime = millis();
        Serial.println("→ STATE_SAFE_STOP (lane switch complete)");
      }
      break;

    // ─────────────────────────────────────────────────────────────
    case STATE_SAFE_STOP:
    // ─────────────────────────────────────────────────────────────
    /*
     * Post-maneuver safety check
     * Only stops if rear is clear (prevents rear-end collision)
     * Otherwise maintains slow speed
     */
      if (dBack > REAR_CLEARANCE) {
        stopAllMotors();
        systemState = STATE_DRIVING;
        Serial.println("→ STATE_DRIVING (safe stop complete)");
      } else {
        runAllMotors(SLOW_SPEED);  // Keep moving if vehicle behind
      }
      break;

    // ─────────────────────────────────────────────────────────────
    case STATE_EMERGENCY_STOP:
    // ─────────────────────────────────────────────────────────────
    /*
     * Highest priority state - overrides all others
     * Immediately stops motors and activates buzzer
     * Includes timeout escape mechanism to prevent deadlock
     */
      stopAllMotors();
      digitalWrite(BUZZER_PIN, HIGH);
      
      unsigned long emergencyDuration = millis() - stateStartTime;
      
      // Timeout mechanism: prevent infinite emergency state
      // If emergency lasts >10 seconds, assume sensor failure
      if (emergencyDuration > EMERGENCY_TIMEOUT) {
        Serial.println("CRITICAL: Emergency timeout - forcing manual mode");
        autopilotEnabled = false;  // Disable autopilot (requires manual intervention)
        digitalWrite(BUZZER_PIN, LOW);
        break;
      }
      
      // Exit emergency state when all sensors show safe distances
      if (dFront >= EMERGENCY_FRONT && 
          dBack >= EMERGENCY_BACK && 
          dRight >= EMERGENCY_SIDE) {
        Serial.println("✓ Emergency cleared - resuming");
        systemState = STATE_DRIVING;
        stateStartTime = millis();
        digitalWrite(BUZZER_PIN, LOW);
      }
      break;
  }

  // ───────────────────────────────────────────────────────────────
  // BUZZER MANAGEMENT
  // ───────────────────────────────────────────────────────────────
  // Turn off buzzer when not in emergency state
  
  if (!immediateDanger && systemState != STATE_EMERGENCY_STOP) {
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// ═══════════════════════════════════════════════════════════════════
// ULTRASONIC DISTANCE MEASUREMENT
// ═══════════════════════════════════════════════════════════════════

long getDistance(int trigPin, int echoPin) {
  /*
   * Measures distance using HC-SR04 ultrasonic sensor
   * 
   * Process:
   * 1. Send 10μs trigger pulse
   * 2. Sensor emits 8 ultrasonic pulses at 40kHz
   * 3. Measure echo pulse duration
   * 4. Calculate distance using speed of sound (343 m/s)
   * 
   * Returns: Distance in centimeters
   */
  
  // Clear trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Send 10μs pulse to trigger ultrasonic burst
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure echo pulse duration (timeout after 10ms)
  // 10ms timeout = max ~170cm range
  unsigned long duration = pulseIn(echoPin, HIGH, 10000);
  
  // No echo received (timeout) - assume maximum distance
  if (duration == 0) {
    return MAX_SENSOR_DISTANCE;
  }
  
  // Calculate distance in centimeters
  // Formula: distance = (time × speed_of_sound) / 2
  // Speed of sound = 343 m/s = 0.0343 cm/μs
  // Divide by 2 because sound travels to object and back
  long distance = duration * 0.0343 / 2;
  
  // Clamp to valid range
  if (distance < MIN_SENSOR_DISTANCE) distance = MIN_SENSOR_DISTANCE;
  if (distance > MAX_SENSOR_DISTANCE) distance = MAX_SENSOR_DISTANCE;
  
  return distance;
}

// ═══════════════════════════════════════════════════════════════════
// MOTOR CONTROL FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

void runAllMotors(int pwmVal) {
  /*
   * Sets all four motors to same speed in forward direction
   * Uses PWM to control speed (0 = stop, 255 = full speed)
   * 
   * Args:
   *   pwmVal: PWM duty cycle (0-255)
   *           NORMAL_SPEED = 110 (43% throttle)
   *           SLOW_SPEED = 80 (31% throttle)
   */
  
  // Clamp PWM value to valid range
  if (pwmVal < 0) pwmVal = 0;
  if (pwmVal > 255) pwmVal = 255;
  
  // Set direction pins for forward motion
  // HIGH/LOW combination determines direction via H-bridge
  digitalWrite(Motor1In1, LOW);  digitalWrite(Motor1In2, HIGH);  // Front Left forward
  digitalWrite(Motor1In3, HIGH); digitalWrite(Motor1In4, LOW);   // Front Right forward
  digitalWrite(Motor2In1, LOW);  digitalWrite(Motor2In2, HIGH);  // Rear Left forward
  digitalWrite(Motor2In3, HIGH); digitalWrite(Motor2In4, LOW);   // Rear Right forward

  // Apply PWM speed control to all motors
  ledcWrite(PWM_CHANNEL_RL, pwmVal);
  ledcWrite(PWM_CHANNEL_RR, pwmVal);
  ledcWrite(PWM_CHANNEL_FL, pwmVal);
  ledcWrite(PWM_CHANNEL_FR, pwmVal);
}

void stopAllMotors() {
  /*
   * Immediately stops all motors
   * Sets all direction pins LOW and PWM to 0
   * Used for emergency stops and manual mode
   */
  
  // Disable all direction control pins (brake mode)
  digitalWrite(Motor1In1, LOW); digitalWrite(Motor1In2, LOW);
  digitalWrite(Motor1In3, LOW); digitalWrite(Motor1In4, LOW);
  digitalWrite(Motor2In1, LOW); digitalWrite(Motor2In2, LOW);
  digitalWrite(Motor2In3, LOW); digitalWrite(Motor2In4, LOW);

  // Set all PWM duty cycles to 0 (no power to motors)
  ledcWrite(PWM_CHANNEL_RL, 0);
  ledcWrite(PWM_CHANNEL_RR, 0);
  ledcWrite(PWM_CHANNEL_FL, 0);
  ledcWrite(PWM_CHANNEL_FR, 0);
}

void gentleTurnRight(int speed) {
  /*
   * Executes gentle right turn using differential steering
   * Left motors run at full speed, right motors at 60% speed
   * Creates smooth arc instead of sharp turn
   * 
   * Args:
   *   speed: Base speed for left motors (right will be 60% of this)
   */
  
  // Clamp speed to valid range
  if (speed < 0) speed = 0;
  if (speed > 255) speed = 255;
  
  // Calculate reduced speed for right motors (60% of base)
  int reduced = speed * 0.6;
  
  // Set all motors to forward direction
  digitalWrite(Motor1In1, HIGH); digitalWrite(Motor1In2, LOW);
  digitalWrite(Motor1In3, HIGH); digitalWrite(Motor1In4, LOW);
  digitalWrite(Motor2In1, HIGH); digitalWrite(Motor2In2, LOW);
  digitalWrite(Motor2In3, HIGH); digitalWrite(Motor2In4, LOW);

  // Differential speeds create turning motion
  ledcWrite(PWM_CHANNEL_RL, speed);     // Left motors: full speed
  ledcWrite(PWM_CHANNEL_RR, reduced);   // Right motors: 60% speed
  ledcWrite(PWM_CHANNEL_FL, speed);     // (causes right turn)
  ledcWrite(PWM_CHANNEL_FR, reduced);
}
