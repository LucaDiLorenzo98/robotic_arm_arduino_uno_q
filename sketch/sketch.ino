#include <Arduino_RouterBridge.h>
#include <Servo.h>
#include <SCServo.h>
#if __has_include(<Modulino_LED_Matrix.h>)
#include <Modulino_LED_Matrix.h>
#define HAS_MODULINO_MATRIX 1
#else
#define HAS_MODULINO_MATRIX 0
#endif

#define SERVO_PIN 9
#define PRESSURE_PIN_A0 A0
#define PRESSURE_PIN_A1 A1

Servo servo;
SMS_STS sts;  // STS3215 serial bus controller

#if HAS_MODULINO_MATRIX
ModulinoLEDMatrix modulinoMatrix;
bool modulino_matrix_ready = false;
#endif
int led_matrix_intensity = 3;  // 0..15, API compatibility

// Called from Python when a face is detected. Angle in degrees (0-180).
void move_servo(int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  servo.write(angle);
}

// Pressure sensors: analog read (0-1023). Exposed via Bridge.
int read_pressure_a0() {
  return analogRead(PRESSURE_PIN_A0);
}

int read_pressure_a1() {
  return analogRead(PRESSURE_PIN_A1);
}

// ---- STS3215 servo bus (SCServo library) ----
int sts_ping(int id) {
  return sts.Ping((u8)id);
}

int sts_read_pos(int id) {
  return sts.ReadPos((u8)id);
}

int sts_move_pos(int id, int position, int speed, int acc) {
  return sts.WritePosEx((u8)id, (s16)position, (u16)speed, (u8)acc);
}

// ---- Modulino LED Matrix over QWIIC ----
int led_matrix_available() {
#if HAS_MODULINO_MATRIX
  return modulino_matrix_ready ? 1 : 0;
#else
  return 0;
#endif
}

int led_matrix_set_intensity(int level) {
#if HAS_MODULINO_MATRIX
  if (level < 0) level = 0;
  if (level > 15) level = 15;
  led_matrix_intensity = level;
  // ModulinoLEDMatrix does not expose a direct brightness API.
  return 1;
#else
  (void)level;
  return -1;
#endif
}

int led_matrix_clear() {
#if HAS_MODULINO_MATRIX
  if (!modulino_matrix_ready) return -1;
  modulinoMatrix.clear();
  return 1;
#else
  return -1;
#endif
}

#if HAS_MODULINO_MATRIX
const uint32_t FRAME_DETECT[3] = {
  0b00000000000000000000000000000000,
  0b00001100000000000000000000110000,
  0b00000000000000000000000000000000
};
const uint32_t FRAME_GRAB[3] = {
  0b00000000011110000001111000000000,
  0b00000000011110000001111000000000,
  0b00000000011110000001111000000000
};
const uint32_t FRAME_RELEASE[3] = {
  0b00000011111111111111111111000000,
  0b00000011111111111111111111000000,
  0b00000011111111111111111111000000
};
#endif

int led_matrix_set_state(int state_code) {
#if HAS_MODULINO_MATRIX
  if (!modulino_matrix_ready) return -1;
  // 0=detect, 1=grab, 2=release
  if (state_code == 0) {
    modulinoMatrix.loadFrame(FRAME_DETECT);
    return 1;
  }
  if (state_code == 1) {
    modulinoMatrix.loadFrame(FRAME_GRAB);
    return 1;
  }
  if (state_code == 2) {
    modulinoMatrix.loadFrame(FRAME_RELEASE);
    return 1;
  }
  return 0;
#else
  (void)state_code;
  return -1;
#endif
}

void setup() {
  Bridge.begin();

  // Standard PWM servo on D9 (gripper)
  servo.attach(SERVO_PIN);
  servo.write(0);

  // STS3215 serial bus on Serial (UNO Q side)
  Serial.begin(1000000);
  sts.pSerial = &Serial;

#if HAS_MODULINO_MATRIX
  if (modulinoMatrix.begin()) {
    modulino_matrix_ready = true;
    modulinoMatrix.clear();
  }
#endif

  pinMode(PRESSURE_PIN_A0, INPUT);
  pinMode(PRESSURE_PIN_A1, INPUT);

  Bridge.provide_safe("move_servo", move_servo);
  Bridge.provide_safe("read_pressure_a0", read_pressure_a0);
  Bridge.provide_safe("read_pressure_a1", read_pressure_a1);
  Bridge.provide_safe("sts_ping", sts_ping);
  Bridge.provide_safe("sts_read_pos", sts_read_pos);
  Bridge.provide_safe("sts_move_pos", sts_move_pos);
  Bridge.provide_safe("led_matrix_available", led_matrix_available);
  Bridge.provide_safe("led_matrix_set_intensity", led_matrix_set_intensity);
  Bridge.provide_safe("led_matrix_clear", led_matrix_clear);
  Bridge.provide_safe("led_matrix_set_state", led_matrix_set_state);
}

void loop() {}
