#include <Arduino.h>

// ─── Ultrasonic Sensor Pins ───────────────────────────────
#define UltraTriggerFront 2
#define UltraEchoFront    17
#define UltraTriggerRight 18  
#define UltraEchoRight    34
#define UltraTriggerBack  21
#define UltraEchoBack     5

// ─── Motor Driver 2 (Rear) ────────────────────────────────
#define Motor2EnA 4    // Rear Left PWM speed control
#define Motor2EnB 12   // Rear Right PWM speed control
#define Motor2In1 22   // Rear Left motor direction control 1
#define Motor2In2 15   // Rear Left motor direction control 2
#define Motor2In3 23   // Rear Right motor direction control 1
#define Motor2In4 16   // Rear Right motor direction control 2

// ─── Motor Driver 1 (Front) ────────────────────────────────
#define Motor1EnA 32   // Front Left PWM speed control
#define Motor1EnB 33   // Front Right PWM speed control
#define Motor1In1 25   // Front Left motor direction control 1
#define Motor1In2 26   // Front Left motor direction control 2
#define Motor1In3 27   // Front Right motor direction control 1
#define Motor1In4 14   // Front Right motor direction control 2

/* ===== OTHER PINS ===== */
#define BUZZER_PIN 13   // Buzzer warning system pin

/* ===== PWM CHANNELS ===== */
#define PWM_CHANNEL_RL 0  // Rear Left
#define PWM_CHANNEL_RR 1  // Rear Right
#define PWM_CHANNEL_FL 2  // Front Left
#define PWM_CHANNEL_FR 3  // Front Right

/* ===== GLOBAL VARIABLES ===== */
bool autopilotEnabled = false;    // start OFF
String rxBuf = "";                // incoming serial buffer
unsigned long lastLoopTime = 0;
const int loopInterval = 500;

/* ===== LANE CONTROL PARAMETERS ===== */
const int LANE_WIDTH = 30;
const int CAR_WIDTH = 22;
const int CAR_LENGTH = 35;
const int ROAD_LENGTH = 300;
int currentLane = 1;  // 1 = left lane, 2 = right lane

/* ===== STATE MACHINE ===== */
enum SystemState { 
  STATE_DRIVING,
  STATE_CHECKING_LANE,
  STATE_SWITCHING_LANE,
  STATE_SAFE_STOP,
  STATE_EMERGENCY_STOP
};
SystemState systemState = STATE_DRIVING;
unsigned long stateStartTime = 0;

/* ===== TIMING CONSTANTS (ms) ===== */
const int LANE_CHECK_TIME = 1000;
const int LANE_SWITCH_TIME = 1500;
const int SLOW_SPEED = 80;
const int NORMAL_SPEED = 110;

/* ===== SAFETY THRESHOLDS (cm) ===== */
const int EMERGENCY_FRONT = 25;
const int EMERGENCY_BACK = 10;
const int EMERGENCY_SIDE = 15;
const int LANE_CLEARANCE = 20;
const int REAR_CLEARANCE = 20;

/* ===== FORWARD DECLARATIONS ===== */
long getDistance(int trigPin, int echoPin);
void runAllMotors(int pwmVal);
void stopAllMotors();
void gentleTurnRight(int speed);
void handleCommand(const String &cmd);

/* ===== SETUP ===== */
void setup() {
  Serial.begin(115200);
  Serial.println("=== SYSTEM INITIALIZATION STARTED ===");

  // Ultrasonic sensor setup
  pinMode(UltraTriggerFront, OUTPUT); pinMode(UltraEchoFront, INPUT);
  pinMode(UltraTriggerRight, OUTPUT); pinMode(UltraEchoRight, INPUT);
  pinMode(UltraTriggerBack, OUTPUT);  pinMode(UltraEchoBack, INPUT);
  Serial.println("Ultrasonic sensors initialized");

  // Motor control setup
  pinMode(Motor2In1, OUTPUT); pinMode(Motor2In2, OUTPUT);
  pinMode(Motor2In3, OUTPUT); pinMode(Motor2In4, OUTPUT);
  pinMode(Motor1In1, OUTPUT); pinMode(Motor1In2, OUTPUT);
  pinMode(Motor1In3, OUTPUT); pinMode(Motor1In4, OUTPUT);
  Serial.println("Motor control pins initialized");

  // Buzzer setup
  pinMode(BUZZER_PIN, OUTPUT);
  Serial.println("Buzzer initialized");

  // PWM setup for ESP32
  ledcSetup(PWM_CHANNEL_RL, 5000, 8);
  ledcSetup(PWM_CHANNEL_RR, 5000, 8);
  ledcSetup(PWM_CHANNEL_FL, 5000, 8);
  ledcSetup(PWM_CHANNEL_FR, 5000, 8);
  ledcAttachPin(Motor2EnA, PWM_CHANNEL_RL);
  ledcAttachPin(Motor2EnB, PWM_CHANNEL_RR);
  ledcAttachPin(Motor1EnA, PWM_CHANNEL_FL);
  ledcAttachPin(Motor1EnB, PWM_CHANNEL_FR);
  Serial.println("PWM channels configured");

  Serial.println("=== SYSTEM INITIALIZATION COMPLETE ===");
  Serial.println("Autopilot status: DISABLED");
  Serial.println("Initial lane: LEFT");
  Serial.println("Road length: 2 meters");
}

/* ===== SERIAL COMMAND HANDLING ===== */
void handleCommand(const String &cmd) {
  if (cmd == "autopilot mode:on") {
    autopilotEnabled = true;
    Serial.println("ACK:autopilot enabled");
  }
  else if (cmd == "autopilot mode:off") {
    autopilotEnabled = false;
    Serial.println("ACK:autopilot disabled");
  }
}

/* ===== MAIN LOOP ===== */
void loop() {
  // —––– Serial read & parse commands –––––
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      rxBuf.trim();
      handleCommand(rxBuf);
      rxBuf = "";
    } else {
      rxBuf += c;
    }
  }

  // —––– Throttle loop frequency —––––
  if (millis() - lastLoopTime < loopInterval) return;
  lastLoopTime = millis();

  // —––– If autopilot is OFF, stop everything —––––
  if (!autopilotEnabled) {
    stopAllMotors();
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  // —––– Read sensors —––––
  long dFront = getDistance(UltraTriggerFront, UltraEchoFront);
  long dRight = getDistance(UltraTriggerRight, UltraEchoRight);
  long dBack  = getDistance(UltraTriggerBack,  UltraEchoBack);
  Serial.print("Distances (cm): Front=");
Serial.print(dFront);
Serial.print(", Right=");
Serial.print(dRight);
Serial.print(", Back=");
Serial.println(dBack);

  // —––– Check for emergency —––––
  bool immediateDanger = (dFront < EMERGENCY_FRONT)
                      || (dBack  < EMERGENCY_BACK)
                      || (dRight < EMERGENCY_SIDE);
  if (immediateDanger) {
    systemState = STATE_EMERGENCY_STOP;
    Serial.println("!!! EMERGENCY STOP TRIGGERED !!!");
  }

  // —––– State Machine —––––
  switch (systemState) {
    case STATE_DRIVING:
      runAllMotors(NORMAL_SPEED);
      if (currentLane == 1 && dRight > LANE_CLEARANCE) {
        systemState = STATE_CHECKING_LANE;
        stateStartTime = millis();
        Serial.println("Switching to CHECKING_LANE");
      }
      break;

    case STATE_CHECKING_LANE:
      runAllMotors(SLOW_SPEED);
      if (dRight > LANE_CLEARANCE && millis() - stateStartTime > LANE_CHECK_TIME) {
        systemState = STATE_SWITCHING_LANE;
        stateStartTime = millis();
        Serial.println("Confirmed: SWITCHING_LANE");
      }
      else if (dRight <= LANE_CLEARANCE) {
        systemState = STATE_DRIVING;
        Serial.println("Abort: back to DRIVING");
      }
      break;

    case STATE_SWITCHING_LANE:
      gentleTurnRight(SLOW_SPEED);
      if (millis() - stateStartTime > LANE_SWITCH_TIME) {
        currentLane = 2;
        systemState = STATE_SAFE_STOP;
        Serial.println("Lane switched: STATE_SAFE_STOP");
      }
      break;

    case STATE_SAFE_STOP:
      if (dBack > REAR_CLEARANCE) {
        stopAllMotors();
        systemState = STATE_DRIVING;
        Serial.println("Safe stop complete: back to DRIVING");
      } else {
        runAllMotors(SLOW_SPEED);
      }
      break;

    case STATE_EMERGENCY_STOP:
  stopAllMotors();
  digitalWrite(BUZZER_PIN, HIGH);
  // Check if all distances are safe now, then resume
  if (dFront >= EMERGENCY_FRONT && dBack >= EMERGENCY_BACK && dRight >= EMERGENCY_SIDE) {
    Serial.println("Emergency cleared: Resuming driving");
    systemState = STATE_DRIVING;
    digitalWrite(BUZZER_PIN, LOW);
  }
  break;
  }

  // —––– Buzzer off if no danger —––––
  if (!immediateDanger) digitalWrite(BUZZER_PIN, LOW);
}

/* ===== ULTRASONIC DISTANCE MEASUREMENT ===== */
long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000);
  return (duration == 0) ? 999 : duration * 0.0343 / 2;
}

/* ===== MOTOR CONTROL ===== */
void runAllMotors(int pwmVal) {
  // Forward direction pins
  digitalWrite(Motor1In1, LOW);  digitalWrite(Motor1In2, HIGH);
  digitalWrite(Motor1In3, HIGH); digitalWrite(Motor1In4, LOW);
  digitalWrite(Motor2In1, LOW);  digitalWrite(Motor2In2, HIGH);
  digitalWrite(Motor2In3, HIGH); digitalWrite(Motor2In4, LOW);

  ledcWrite(PWM_CHANNEL_RL, pwmVal);
  ledcWrite(PWM_CHANNEL_RR, pwmVal);
  ledcWrite(PWM_CHANNEL_FL, pwmVal);
  ledcWrite(PWM_CHANNEL_FR, pwmVal);
}

void stopAllMotors() {
  digitalWrite(Motor1In1, LOW); digitalWrite(Motor1In2, LOW);
  digitalWrite(Motor1In3, LOW); digitalWrite(Motor1In4, LOW);
  digitalWrite(Motor2In1, LOW); digitalWrite(Motor2In2, LOW);
  digitalWrite(Motor2In3, LOW); digitalWrite(Motor2In4, LOW);

  ledcWrite(PWM_CHANNEL_RL, 0);
  ledcWrite(PWM_CHANNEL_RR, 0);
  ledcWrite(PWM_CHANNEL_FL, 0);
  ledcWrite(PWM_CHANNEL_FR, 0);
}

void gentleTurnRight(int speed) {
  int reduced = speed * 0.6;
  // All motors forward
  digitalWrite(Motor1In1, HIGH); digitalWrite(Motor1In2, LOW);
  digitalWrite(Motor1In3, HIGH); digitalWrite(Motor1In4, LOW);
  digitalWrite(Motor2In1, HIGH); digitalWrite(Motor2In2, LOW);
  digitalWrite(Motor2In3, HIGH); digitalWrite(Motor2In4, LOW);

  // Differential speeds
  ledcWrite(PWM_CHANNEL_RL, speed);
  ledcWrite(PWM_CHANNEL_RR, reduced);
  ledcWrite(PWM_CHANNEL_FL, reduced);
  ledcWrite(PWM_CHANNEL_FR, speed);
}