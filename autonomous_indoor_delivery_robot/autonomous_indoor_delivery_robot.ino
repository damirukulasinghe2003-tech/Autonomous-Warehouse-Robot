// ============================================================
//  Autonomous Indoor Delivery Robot — v3.0
//  Target Board : Arduino Uno
//  Author       : Project Team
//  Date         : 2026-11-18
//
//  Hardware Summary
//  ─────────────────────────────────────────────────────────
//  5-Ch IR Sensor Array → A0 (L) .. A4 (R)
//  L298N Motor Driver   → IN1=D8, IN2=D9, IN3=D10, IN4=D11
//                         ENA / ENB jumpers ON  → full voltage, digital only
//                         Motor speed = 100 % via direct digitalWrite()
//  SG90 Servo           → Signal = D3
//  HC-SR04 Ultrasonic   → TRIG=D5, ECHO=D4
// ============================================================

#include <Servo.h>

// ============================================================
//  ★  CONFIGURABLE SETTINGS  ★
// ============================================================

// IR sensor line logic
// LOW  → sensor outputs LOW  when it sees the black line (default)
// HIGH → change to HIGH if your module has inverted output
#define LINE_DETECTED    LOW

// Obstacle detection
#define OBSTACLE_DIST_CM 20     // cm — stop & scan when obstacle is closer than this

// Mission timing
#define DEST_WAIT_MS     5000   // ms — pause at destination before U-turn
#define UTURN_BLIND_MS   700    // ms — minimum blind spin before checking IR

// Serial debug throttle
#define SERIAL_INTERVAL_MS 150  // ms — how often to print sensor data

// ============================================================
//  PIN DEFINITIONS  (do not change — matches your wiring)
// ============================================================

// 5-channel IR sensor array (analog pins used as digital inputs)
const int IR_L  = A0;   // OUT1 — Leftmost
const int IR_LC = A1;   // OUT2 — Left-Center
const int IR_C  = A2;   // OUT3 — Center
const int IR_RC = A3;   // OUT4 — Right-Center
const int IR_R  = A4;   // OUT5 — Rightmost

// L298N motor driver
const int IN1 = 8;      // Left  motor — forward wire
const int IN2 = 9;      // Left  motor — backward wire
const int IN3 = 10;     // Right motor — forward wire
const int IN4 = 11;     // Right motor — backward wire

// Servo & ultrasonic
const int SERVO_PIN = 3;
const int TRIG_PIN  = 5;
const int ECHO_PIN  = 4;

// Servo angle constants
const int SERVO_CENTER = 90;
const int SERVO_LEFT   = 150;
const int SERVO_RIGHT  = 30;

// ============================================================
//  STATE MACHINE DEFINITION
// ============================================================

enum RobotState {
  GOING_TO_DEST,     // Following black line toward destination
  TURNING_AROUND,    // Performing 180° U-turn at destination
  RETURNING_HOME,    // Following black line back to start
  MISSION_COMPLETE   // Final stop — robot stays idle
};

RobotState robotState = GOING_TO_DEST;

// ============================================================
//  GLOBALS
// ============================================================

Servo scanServo;

// Non-blocking ultrasonic cache
unsigned long lastUltrasonicMs = 0;
int           cachedDistCm     = 999;

// Non-blocking serial throttle
unsigned long lastSerialMs = 0;

// ============================================================
//  MOTOR CONTROL — Direct digitalWrite(), full voltage
//
//  ENA / ENB jumpers are installed on the L298N, so the enable
//  pins are permanently HIGH.  Motor speed is therefore always
//  100 %.  Using analogWrite() or pulsing IN1–IN4 at anything
//  less than full duty would drop below starting torque —
//  that is exactly why the wheels stopped spinning in v2.
//  Solution: plain, direct digitalWrite() only.
// ============================================================

/**
 * @brief Both motors forward — robot drives straight ahead.
 */
void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // Left  → forward
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);   // Right → forward
}

/**
 * @brief Gentle left correction (A1 triggered).
 *        Left motor stopped, right motor forward → pivot left slowly.
 */
void steerLeft() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);   // Left  → stopped
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);   // Right → forward
}

/**
 * @brief Sharp left turn (A0 triggered — robot is well off-line to the right).
 *        Left motor backward, right motor forward → tight counter-clockwise spin.
 */
void turnLeftSharp() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);  // Left  → backward
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);   // Right → forward
}

/**
 * @brief Gentle right correction (A3 triggered).
 *        Left motor forward, right motor stopped → pivot right slowly.
 */
void steerRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // Left  → forward
  digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);   // Right → stopped
}

/**
 * @brief Sharp right turn (A4 triggered — robot is well off-line to the left).
 *        Left motor forward, right motor backward → tight clockwise spin.
 */
void turnRightSharp() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // Left  → forward
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);  // Right → backward
}

/**
 * @brief All four IN pins LOW — motors coast to a stop.
 */
void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

/**
 * @brief Spin in place counter-clockwise (left fwd + right rev).
 *        Used for the 180° U-turn at destination.
 */
void spinCCW() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // Left  → forward
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);  // Right → backward
}

// ============================================================
//  ULTRASONIC SENSOR
// ============================================================

/**
 * @brief  Fire the HC-SR04 and return distance in cm.
 *
 *  Key fix: duration == 0 means pulseIn() timed out (no echo / out of range).
 *  Returning 999 ensures this is always > OBSTACLE_DIST_CM and never
 *  triggers a false obstacle stop.
 *
 *  Timeout = 25 ms → maximum measurable range ≈ 215 cm.
 *  The call never blocks longer than 25 ms.
 */
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000UL);  // 25 ms timeout

  if (duration == 0) return 999;   // no echo = no obstacle in range
  return (int)(duration / 58L);    // µs / 58 = cm
}

/**
 * @brief  Non-blocking ultrasonic update.
 *         Pings the sensor at most once every 80 ms and stores
 *         the result in cachedDistCm so the main loop is not blocked.
 */
void updateUltrasonic() {
  if (millis() - lastUltrasonicMs >= 80UL) {
    cachedDistCm     = getDistance();
    lastUltrasonicMs = millis();
  }
}

// ============================================================
//  OBSTACLE SCAN & AVOIDANCE
// ============================================================

/**
 * @brief  Called when cachedDistCm drops below OBSTACLE_DIST_CM.
 *
 *  Sequence:
 *   1. Stop motors.
 *   2. Servo → 150° (left),  wait 450 ms, read leftDist.
 *   3. Servo →  30° (right), wait 450 ms, read rightDist.
 *   4. Servo →  90° (center),wait 300 ms.
 *   5. Turn toward the clearer side (400 ms).
 *   6. Drive forward past the obstacle (500 ms).
 *   7. Turn back toward the line (400 ms).
 *   8. Drive until any IR sensor re-acquires the black line (3 s timeout).
 */
void scanAndAvoidObstacle() {
  Serial.println(F("[AVOID] Obstacle detected! Stopping."));
  stopMotors();
  delay(300);

  // ── Scan LEFT ─────────────────────────────────────────
  scanServo.write(SERVO_LEFT);
  delay(450);
  int leftDist = getDistance();
  Serial.print(F("[AVOID] Left="));  Serial.print(leftDist);  Serial.println(F("cm"));

  // ── Scan RIGHT ────────────────────────────────────────
  scanServo.write(SERVO_RIGHT);
  delay(450);
  int rightDist = getDistance();
  Serial.print(F("[AVOID] Right=")); Serial.print(rightDist); Serial.println(F("cm"));

  // ── Return servo to centre ────────────────────────────
  scanServo.write(SERVO_CENTER);
  delay(300);

  // ── Choose the side with more clearance ───────────────
  bool goLeft = (leftDist >= rightDist);
  Serial.print(F("[AVOID] Turning "));
  Serial.println(goLeft ? F("LEFT") : F("RIGHT"));

  // ── Phase 1: Turn away from obstacle (400 ms) ─────────
  if (goLeft) { steerLeft(); } else { steerRight(); }
  delay(400);

  // ── Phase 2: Drive forward past obstacle (500 ms) ─────
  moveForward();
  delay(500);

  // ── Phase 3: Turn back toward line (400 ms) ───────────
  if (goLeft) { steerRight(); } else { steerLeft(); }
  delay(400);

  // ── Phase 4: Drive until IR re-acquires line (3 s max) ─
  Serial.println(F("[AVOID] Re-acquiring line..."));
  unsigned long t0 = millis();
  while (millis() - t0 < 3000UL) {
    moveForward();
    bool anyLine = (digitalRead(IR_L)  == LINE_DETECTED) ||
                   (digitalRead(IR_LC) == LINE_DETECTED) ||
                   (digitalRead(IR_C)  == LINE_DETECTED) ||
                   (digitalRead(IR_RC) == LINE_DETECTED) ||
                   (digitalRead(IR_R)  == LINE_DETECTED);
    if (anyLine) {
      Serial.println(F("[AVOID] Line re-acquired!"));
      break;
    }
    delay(20);
  }

  stopMotors();
  delay(100);
}

// ============================================================
//  LINE FOLLOWING  — 5-level steering
// ============================================================

/**
 * @brief  Read all 5 IR sensors, apply the correct motor command, and
 *         return true when the end of the tape is reached.
 *
 *  End-of-tape detection: all 5 sensors see NO line simultaneously.
 *  This means the robot has driven past the last piece of tape and
 *  is now on bare floor — that is the destination / home trigger.
 *
 *  Steering priority while tape is present (highest → lowest urgency):
 *
 *   Condition          Sensor(s) active   Action
 *   ─────────────────────────────────────────────────────────
 *   Sharp left needed  A0 (leftmost)     turnLeftSharp()
 *   Gentle left        A1                steerLeft()
 *   Gentle right       A3                steerRight()
 *   Sharp right needed A4 (rightmost)    turnRightSharp()
 *   On track           A2 centre only    moveForward()
 *   End of tape        ALL 5 off line    stopMotors(), return true
 *
 * @return  true when all 5 sensors detect NO line (end of tape reached).
 */
bool followLine() {
  bool sL  = (digitalRead(IR_L)  == LINE_DETECTED);  // A0
  bool sLC = (digitalRead(IR_LC) == LINE_DETECTED);  // A1
  bool sC  = (digitalRead(IR_C)  == LINE_DETECTED);  // A2
  bool sRC = (digitalRead(IR_RC) == LINE_DETECTED);  // A3
  bool sR  = (digitalRead(IR_R)  == LINE_DETECTED);  // A4

  // ── Sharp left (A0 dominates) ─────────────────────────
  if (sL)       { turnLeftSharp();  return false; }

  // ── Gentle left (A1) ──────────────────────────────────
  if (sLC)      { steerLeft();      return false; }

  // ── Gentle right (A3) ─────────────────────────────────
  if (sRC)      { steerRight();     return false; }

  // ── Sharp right (A4 dominates) ────────────────────────
  if (sR)       { turnRightSharp(); return false; }

  // ── Centre only → straight ahead ──────────────────────
  if (sC)       { moveForward();    return false; }

  // ── ALL 5 sensors off line → end of tape reached ──────
  // No sensor sees the line: the robot has followed the tape to its
  // physical end and is now on bare floor. Stop and signal the caller.
  stopMotors();
  return true;
}

// ============================================================
//  U-TURN  (180° spin-in-place at destination)
// ============================================================

/**
 * @brief  Spin counter-clockwise in place until the center sensor A2
 *         re-acquires the black line (now pointing back toward home).
 *
 *  Phase 1 — Blind spin: ignore IR for UTURN_BLIND_MS (700 ms).
 *             Ensures the robot has cleared the destination marker
 *             before it starts checking the centre IR sensor.
 *
 *  Phase 2 — Guided spin: keep spinning until A2 sees the line,
 *             with a 3000 ms safety timeout.
 */
void doUTurn() {
  Serial.println(F("[UTURN] Starting 180 deg U-turn..."));

  // Phase 1: blind spin — do not check IR yet
  spinCCW();
  delay(UTURN_BLIND_MS);

  // Phase 2: guided spin — wait for centre sensor to find the return line
  unsigned long tOut = millis();
  while (millis() - tOut < 3000UL) {
    spinCCW();  // keep calling so it stays running in the while loop
    if (digitalRead(IR_C) == LINE_DETECTED) {
      Serial.println(F("[UTURN] Centre IR found line!"));
      break;
    }
    delay(10);
  }

  stopMotors();
  delay(150);
  Serial.println(F("[UTURN] Complete."));
}

// ============================================================
//  SERIAL DEBUG PRINTER  (non-blocking — throttled)
// ============================================================

void printDebug() {
  if (millis() - lastSerialMs < (unsigned long)SERIAL_INTERVAL_MS) return;
  lastSerialMs = millis();

  bool sL  = (digitalRead(IR_L)  == LINE_DETECTED);
  bool sLC = (digitalRead(IR_LC) == LINE_DETECTED);
  bool sC  = (digitalRead(IR_C)  == LINE_DETECTED);
  bool sRC = (digitalRead(IR_RC) == LINE_DETECTED);
  bool sR  = (digitalRead(IR_R)  == LINE_DETECTED);

  Serial.print(F("IR["));
  Serial.print(sL  ? '1' : '0');
  Serial.print(sLC ? '1' : '0');
  Serial.print(sC  ? '1' : '0');
  Serial.print(sRC ? '1' : '0');
  Serial.print(sR  ? '1' : '0');
  Serial.print(F("] US="));
  Serial.print(cachedDistCm);
  Serial.print(F("cm State="));

  switch (robotState) {
    case GOING_TO_DEST:    Serial.println(F("GOING_TO_DEST"));    break;
    case TURNING_AROUND:   Serial.println(F("TURNING_AROUND"));   break;
    case RETURNING_HOME:   Serial.println(F("RETURNING_HOME"));   break;
    case MISSION_COMPLETE: Serial.println(F("MISSION_COMPLETE")); break;
  }
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
  Serial.begin(9600);
  Serial.println(F("================================================"));
  Serial.println(F(" Autonomous Delivery Robot — v3.0 Boot         "));
  Serial.println(F("================================================"));

  // ── Motor pins ────────────────────────────────────────
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();   // ensure motors off at boot

  // ── Ultrasonic pins ───────────────────────────────────
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // ── IR sensor pins (analog used as digital) ───────────
  pinMode(IR_L,  INPUT);
  pinMode(IR_LC, INPUT);
  pinMode(IR_C,  INPUT);
  pinMode(IR_RC, INPUT);
  pinMode(IR_R,  INPUT);

  // ── Servo self-test sweep ─────────────────────────────
  // Visually confirms servo wiring and motion before the robot moves.
  // Sequence: 90 (centre) -> 150 (left) -> 30 (right) -> 90 (centre)
  scanServo.attach(SERVO_PIN);
  Serial.println(F("[SERVO] Self-test: 90 -> 150 -> 30 -> 90"));
  scanServo.write(SERVO_CENTER); delay(400);
  scanServo.write(SERVO_LEFT);   delay(400);
  scanServo.write(SERVO_RIGHT);  delay(400);
  scanServo.write(SERVO_CENTER); delay(400);
  Serial.println(F("[SERVO] Self-test complete. Servo parked at 90."));

  // ── Pre-start delay ───────────────────────────────────
  Serial.println(F("[INIT] Waiting 1500 ms..."));
  delay(1500);

  // ── Start-line clearance drive ────────────────────────
  // Drive forward 400 ms to leave the starting cross-marker so the
  // all-5-sensors station detect does not fire immediately.
  Serial.println(F("[INIT] Start clearance: forward 400 ms..."));
  moveForward();
  delay(400);
  stopMotors();
  delay(100);

  // ── Set initial state ─────────────────────────────────
  robotState = GOING_TO_DEST;
  Serial.println(F("[STATE] -> GOING_TO_DEST"));
  Serial.println(F("[INIT] Mission started!"));
}

// ============================================================
//  MAIN LOOP
// ============================================================

void loop() {
  // ── Non-blocking background tasks ─────────────────────
  updateUltrasonic();   // ping HC-SR04 every 80 ms, store in cachedDistCm
  printDebug();         // print sensor snapshot every 150 ms

  // ── State machine ─────────────────────────────────────
  switch (robotState) {

    // ════════════════════════════════════════════════════
    //  STATE 1 — Drive to destination while following line
    // ════════════════════════════════════════════════════
    case GOING_TO_DEST: {

      // Priority 1: obstacle check
      // cachedDistCm >= 2    → filters sensor noise near zero
      // cachedDistCm == 999  → no echo = clear path, skip avoidance
      if (cachedDistCm >= 2 && cachedDistCm < OBSTACLE_DIST_CM) {
        scanAndAvoidObstacle();
        break;
      }

      // Priority 2: follow the line
      bool atDest = followLine();

      if (atDest) {
        // ── All 5 sensors off line → end of tape reached ──
        Serial.println(F("[STATE] End of tape reached!"));
        stopMotors();

        // ── Wait at destination ─────────────────────────
        Serial.print(F("[STATE] Waiting "));
        Serial.print(DEST_WAIT_MS);
        Serial.println(F(" ms..."));
        delay(DEST_WAIT_MS);

        // ── Perform U-turn ──────────────────────────────
        robotState = TURNING_AROUND;
        Serial.println(F("[STATE] -> TURNING_AROUND"));
        doUTurn();

        // ── Begin return journey ────────────────────────
        robotState = RETURNING_HOME;
        Serial.println(F("[STATE] -> RETURNING_HOME"));
      }
      break;
    }

    // ════════════════════════════════════════════════════
    //  STATE 2 — Turning (doUTurn is called inline above)
    // ════════════════════════════════════════════════════
    case TURNING_AROUND:
      // Entered and exited inside GOING_TO_DEST block above.
      // Defensive stop if we ever land here alone.
      stopMotors();
      break;

    // ════════════════════════════════════════════════════
    //  STATE 3 — Follow line back home
    // ════════════════════════════════════════════════════
    case RETURNING_HOME: {

      bool atHome = followLine();

      if (atHome) {
        Serial.println(F("[STATE] Start of tape reached! Mission complete."));
        stopMotors();
        robotState = MISSION_COMPLETE;
        Serial.println(F("[STATE] -> MISSION_COMPLETE"));
        Serial.println(F("================================================"));
        Serial.println(F(" Round-trip delivery mission accomplished!      "));
        Serial.println(F("================================================"));
      }
      break;
    }

    // ════════════════════════════════════════════════════
    //  STATE 4 — Mission done — stay stopped permanently
    // ════════════════════════════════════════════════════
    case MISSION_COMPLETE:
      stopMotors();
      // printDebug() above keeps printing MISSION_COMPLETE for visibility.
      break;

  } // end switch
}
