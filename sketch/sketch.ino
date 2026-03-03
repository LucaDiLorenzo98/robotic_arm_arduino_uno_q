#include <Arduino_RouterBridge.h>
#include <Servo.h>
#include <SCServo.h>

#define SERVO_PIN 9
#define PRESSURE_PIN_A0 A0
#define PRESSURE_PIN_A1 A1

Servo servo;
SMS_STS sts;  // STS3215 serial bus controller

// Called from Python (MPU) when a face is detected. Angle in degrees (0–180).
void move_servo(int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  servo.write(angle);
}

// Pressure sensors: analog read (0–1023). Exposed to Python via Bridge.
int read_pressure_a0() {
  return analogRead(PRESSURE_PIN_A0);
}

int read_pressure_a1() {
  return analogRead(PRESSURE_PIN_A1);
}

// ---- STS3215 servo bus (SCServo library) ----
// NOTE: bus is wired to the UART used by `Serial` (D0/D1 on typical Arduino-style boards).
// Python controls the arm by calling these functions via Bridge; all protocol details stay on the MCU.

int sts_ping(int id) {
  int res = sts.Ping((u8)id);
  // SCServo Ping returns >=0 on success, <0 on error. We just forward the raw value.
  return res;
}

int sts_read_pos(int id) {
  // Returns raw position value from the servo or <0 on error.
  int pos = sts.ReadPos((u8)id);
  return pos;
}

int sts_move_pos(int id, int position, int speed, int acc) {
  // High-level move command. Position is STS units (typically 0–4095 for 0–360°).
  // Speed and acc are passed through directly.
  int res = sts.WritePosEx((u8)id, (s16)position, (u16)speed, (u8)acc);
  return res;
}

void setup() {
  Bridge.begin();

  // Standard PWM servo on D9 (gripper)
  servo.attach(SERVO_PIN);
  servo.write(0);

  // STS3215 serial bus sulla UART esposta come `Serial` dal core UNO Q
  Serial.begin(1000000);
  sts.pSerial = &Serial;

  pinMode(PRESSURE_PIN_A0, INPUT);
  pinMode(PRESSURE_PIN_A1, INPUT);

  Bridge.provide_safe("move_servo", move_servo);
  Bridge.provide_safe("read_pressure_a0", read_pressure_a0);
  Bridge.provide_safe("read_pressure_a1", read_pressure_a1);

  // STS3215 control endpoints exposed to Python
  Bridge.provide_safe("sts_ping", sts_ping);
  Bridge.provide_safe("sts_read_pos", sts_read_pos);
  Bridge.provide_safe("sts_move_pos", sts_move_pos);
}

void loop() {
}
