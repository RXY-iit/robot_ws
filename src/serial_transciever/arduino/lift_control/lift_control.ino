// lift_control.ino
// Receives serial commands from ROS2 lift_serial_node and controls the
// AZD-KD driver via MOSFET-level GPIO signals (JOG mode).
//
// Commands (newline-terminated):
//   UP   → FW pin HIGH, RV pin LOW  (motor moves up)
//   DOWN → FW pin LOW,  RV pin HIGH (motor moves down)
//   STOP → both LOW                 (motor decelerates and stops)
//
// Wiring:
//   Pin 8 (FW) → MOSFET gate for AZD-KD FW input  (UP  direction)
//   Pin 9 (RV) → MOSFET gate for AZD-KD RV input  (DOWN direction)
//
// AZD-KD driver parameter reference (as configured):
//   JOG speed: 60 mm/s
//   JOG accel: 0.30000 m/s^2
//   Both FW and RV LOW = driver stops (deceleration applied by driver)

const int PIN_FW = 8;
const int PIN_RV = 9;

void setup() {
  Serial.begin(9600);
  pinMode(PIN_FW, OUTPUT);
  pinMode(PIN_RV, OUTPUT);
  // Safe default: both off
  digitalWrite(PIN_FW, LOW);
  digitalWrite(PIN_RV, LOW);
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    applyCommand(cmd);
  }
}

void applyCommand(const String& cmd) {
  if (cmd == "UP") {
    digitalWrite(PIN_RV, LOW);   // clear reverse first to avoid momentary both-HIGH
    digitalWrite(PIN_FW, HIGH);
  } else if (cmd == "DOWN") {
    digitalWrite(PIN_FW, LOW);   // clear forward first
    digitalWrite(PIN_RV, HIGH);
  } else if (cmd == "STOP") {
    digitalWrite(PIN_FW, LOW);
    digitalWrite(PIN_RV, LOW);
  }
  // Unknown commands are silently ignored (safe: pins unchanged)
}
