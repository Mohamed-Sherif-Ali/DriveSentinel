/*
  esp32_autopilot_unified.ino  (MERGED)
  -------------------------------------
  Normalized serial protocol + your PWM/sensor wiring.
  Replace or tune any section as needed.
*/

#include <Arduino.h>

// === Pins from your original PWM.ino ===
#define UltraTriggerFront 2
#define UltraEchoFront    17
#define UltraTriggerRight 18
#define UltraEchoRight    34
#define UltraTriggerBack  21
#define UltraEchoBack     5
#define Motor2EnA 4
#define Motor2EnB 12
#define Motor2In1 22
#define Motor2In2 15
#define Motor2In3 23
#define Motor2In4 16
#define Motor1EnA 32
#define Motor1EnB 33
#define Motor1In1 25
#define Motor1In2 26
#define Motor1In3 27
#define Motor1In4 14
#define BUZZER_PIN 13
#define PWM_CHANNEL_RL 0
#define PWM_CHANNEL_RR 1
#define PWM_CHANNEL_FL 2
#define PWM_CHANNEL_FR 3


// === Tunables (units in cm, ms, %) ===
const int MAX_PWM_PERCENT = 40;     // cap motor power
const int OBST_WARN_CM    = 8;
const int OBST_STOP_CM    = 5;
const unsigned long TELEMETRY_MS = 200;

// === State ===
enum RunState { DRIVING, CHECKING_LANE, SWITCHING_LANE, SAFE_STOP, EMERGENCY };
RunState state = DRIVING;
bool autopilotEnabled = false;
bool buzzerOn = false;
int targetSpeedPct = 30;   // 0..100 (capped by MAX_PWM_PERCENT)

enum LaneStatus { L_LEFT, L_CENTER, L_RIGHT, L_UNKNOWN };
LaneStatus lane = L_UNKNOWN;

// === Distance function from your code ===
long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, 30000);
  return (duration == 0) ? 999 : duration * 0.0343 / 2;
}

// === Init using your wiring ===
void motorsInit() {
  pinMode(Motor1In1, OUTPUT); pinMode(Motor1In2, OUTPUT);
  pinMode(Motor1In3, OUTPUT); pinMode(Motor1In4, OUTPUT);
  pinMode(Motor2In1, OUTPUT); pinMode(Motor2In2, OUTPUT);
  pinMode(Motor2In3, OUTPUT); pinMode(Motor2In4, OUTPUT);

  // PWM channels and attachments
  ledcSetup(PWM_CHANNEL_RL, 5000, 8);
  ledcSetup(PWM_CHANNEL_RR, 5000, 8);
  ledcSetup(PWM_CHANNEL_FL, 5000, 8);
  ledcSetup(PWM_CHANNEL_FR, 5000, 8);

  // Attach channels to enable pins (match your original mapping)
  ledcAttachPin(Motor2EnA, PWM_CHANNEL_RL);
  ledcAttachPin(Motor2EnB, PWM_CHANNEL_RR);
  ledcAttachPin(Motor1EnA, PWM_CHANNEL_FL);
  ledcAttachPin(Motor1EnB, PWM_CHANNEL_FR);
}

void sensorsInit() {
  pinMode(UltraTriggerFront, OUTPUT); pinMode(UltraEchoFront, INPUT);
  pinMode(UltraTriggerRight, OUTPUT); pinMode(UltraEchoRight, INPUT);
  pinMode(UltraTriggerBack,  OUTPUT); pinMode(UltraEchoBack, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
}

// === Motor helpers (percent-based) ===
static inline int pctToPWM(int pct) {
  if (pct < 0) pct = 0;
  if (pct > MAX_PWM_PERCENT) pct = MAX_PWM_PERCENT;
  return map(pct, 0, 100, 0, 255);
}

void setForwardDirection() {
  // All motors forward (adjust if your wiring demands inversions)
  digitalWrite(Motor1In1, HIGH); digitalWrite(Motor1In2, LOW);
  digitalWrite(Motor1In3, HIGH); digitalWrite(Motor1In4, LOW);
  digitalWrite(Motor2In1, HIGH); digitalWrite(Motor2In2, LOW);
  digitalWrite(Motor2In3, HIGH); digitalWrite(Motor2In4, LOW);
}

void motorsSetPercent(int leftPct, int rightPct) {
  setForwardDirection();
  int l = pctToPWM(leftPct);
  int r = pctToPWM(rightPct);
  // Left side: RL + FL ; Right side: RR + FR
  ledcWrite(PWM_CHANNEL_RL, l);
  ledcWrite(PWM_CHANNEL_FL, l);
  ledcWrite(PWM_CHANNEL_RR, r);
  ledcWrite(PWM_CHANNEL_FR, r);
}

void stopAllMotors() {
  ledcWrite(PWM_CHANNEL_RL, 0);
  ledcWrite(PWM_CHANNEL_FL, 0);
  ledcWrite(PWM_CHANNEL_RR, 0);
  ledcWrite(PWM_CHANNEL_FR, 0);
}

void gentleTurnRightPct(int pct) {
  int reduced = (pct * 6) / 10; // ~60%
  setForwardDirection();
  ledcWrite(PWM_CHANNEL_RL, pctToPWM(pct));
  ledcWrite(PWM_CHANNEL_RR, pctToPWM(reduced));
  ledcWrite(PWM_CHANNEL_FL, pctToPWM(reduced));
  ledcWrite(PWM_CHANNEL_FR, pctToPWM(pct));
}

// === Buzzer ===
void setBuzzer(bool on) {
  buzzerOn = on;
  digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
}

void applySpeed() {
  int capped = targetSpeedPct;
  if (capped > MAX_PWM_PERCENT) capped = MAX_PWM_PERCENT;
  if (capped < 0) capped = 0;
  motorsSetPercent(capped, capped);
}

// === Protocol ===
void printState(RunState s) {
  const char* n = "DRIVING";
  if (s==CHECKING_LANE) n="CHECKING_LANE";
  else if (s==SWITCHING_LANE) n="SWITCHING_LANE";
  else if (s==SAFE_STOP) n="SAFE_STOP";
  else if (s==EMERGENCY) n="EMERGENCY";
  Serial.print("STATE:"); Serial.println(n);
}

void printLane(LaneStatus l) {
  const char* n = "UNKNOWN";
  if (l==L_LEFT) n="LEFT";
  else if (l==L_CENTER) n="CENTER";
  else if (l==L_RIGHT) n="RIGHT";
  Serial.print("LANE:"); Serial.println(n);
}

void handleLine(String s){
  s.trim();
  if (s.length() == 0) return;

  if (s.startsWith("CMD:MODE:")) {
    bool on = s.endsWith(":ON");
    autopilotEnabled = on;
    Serial.println(on ? "ACK:MODE:ON" : "ACK:MODE:OFF");
    printState(state);
    return;
  }
  if (s.startsWith("CMD:SPEED:")) {
    int v = s.substring(strlen("CMD:SPEED:")).toInt();
    targetSpeedPct = v;
    applySpeed();
    return;
  }
  if (s.startsWith("CMD:BUZZER:")) {
    bool on = s.endsWith(":ON");
    setBuzzer(on);
    return;
  }
  Serial.println("ERR:UNK_CMD");
}

String inBuf;
void readSerial(){
  while (Serial.available()){
    char c = (char)Serial.read();
    if (c == '\n') { handleLine(inBuf); inBuf = ""; }
    else if (c != '\r') { inBuf += c; }
  }
}

// === Setup/loop ===
unsigned long lastTelem = 0;

void setup(){
  Serial.begin(115200);
  Serial.println("=== ESP32 UNIFIED AUTOPILOT (MERGED) ===");
  motorsInit();
  sensorsInit();
  setBuzzer(false);
  stopAllMotors();
  printState(state);
  Serial.println("ACK:MODE:OFF");
}

void loop(){
  readSerial();

  long dFront = getDistance(UltraTriggerFront, UltraEchoFront);
  long dRight = getDistance(UltraTriggerRight, UltraEchoRight);
  long dBack  = getDistance(UltraTriggerBack,  UltraEchoBack);

  if (autopilotEnabled) {
    if (dFront >= 0 && dFront <= OBST_STOP_CM) {
      stopAllMotors();
      state = SAFE_STOP;
    } else {
      state = DRIVING;
      applySpeed();
    }
    if (dFront >= 0 && dFront <= OBST_WARN_CM) { Serial.print("WARN:OBSTACLE:FRONT:"); Serial.println(dFront); }
    if (dRight >= 0 && dRight <= OBST_WARN_CM) { Serial.print("WARN:OBSTACLE:RIGHT:"); Serial.println(dRight); }
    if (dBack  >= 0 && dBack  <= OBST_WARN_CM) { Serial.print("WARN:OBSTACLE:BACK:");  Serial.println(dBack);  }
  } else {
    stopAllMotors();
  }

  unsigned long now = millis();
  if (now - lastTelem >= TELEMETRY_MS) {
    lastTelem = now;
    Serial.print("DIST:F="); Serial.print(dFront);
    Serial.print(" R="); Serial.print(dRight);
    Serial.print(" B="); Serial.println(dBack);
    printLane(lane);
    printState(state);
  }
}
