#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <Servo.h>
#include <SCServo.h>
#if __has_include(<Modulino.h>)
#include <Modulino.h>
#define HAS_MODULINO_CORE 1
#elif __has_include(<Arduino_Modulino.h>)
c
#define HAS_MODULINO_CORE 1
#else
#define HAS_MODULINO_CORE 0
#endif

#define SERVO_PIN 9
#define PRESSURE_PIN_A0 A0
#define PRESSURE_PIN_A1 A1

Servo servo;
SMS_STS sts;  // STS3215 serial bus controller

#if HAS_MODULINO_CORE
ModulinoPixels modulinoPixels;
bool modulino_pixels_ready = false;
#endif
int led_matrix_intensity = 3;  // 0..15, API compatibility

// Called from Python when a face is detected. Angle in degrees (0-180).
void move_servo(int angle) {
  if (!servo.attached()) {
    servo.attach(SERVO_PIN);
  }
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  servo.write(angle);
}

int gripper_hold(int hold) {
  if (hold) {
    if (!servo.attached()) {
      servo.attach(SERVO_PIN);
    }
  } else {
    if (servo.attached()) {
      servo.detach();
    }
  }
  return 1;
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

int sts_torque_enable(int id, int enable) {
  u8 en = enable ? 1 : 0;
  return sts.EnableTorque((u8)id, en);
}

// ---- Modulino Pixels over QWIIC (treated as 8x13 matrix) ----
#define MATRIX_H 8
#define MATRIX_W 13
#define MATRIX_PIXELS (MATRIX_H * MATRIX_W)

int led_matrix_available() {
#if HAS_MODULINO_CORE
  return modulino_pixels_ready ? 1 : 0;
#else
  return 0;
#endif
}

static int _pixels_brightness() {
  int level = led_matrix_intensity;
  if (level < 0) level = 0;
  if (level > 15) level = 15;
  // Modulino examples use brightness 0..100.
  return (level * 100) / 15;
}

static int _matrix_index(int x, int y) {
  if (x < 0 || x >= MATRIX_W || y < 0 || y >= MATRIX_H) return -1;
  // Serpentine mapping row-by-row:
  // even row: left->right, odd row: right->left
  if ((y & 1) == 0) {
    return (y * MATRIX_W) + x;
  }
  return (y * MATRIX_W) + (MATRIX_W - 1 - x);
}

static void _matrix_set_xy(int x, int y, ModulinoColor color, int brightness) {
  int idx = _matrix_index(x, y);
  if (idx < 0) return;
  modulinoPixels.set(idx, color, brightness);
}

static void _matrix_clear_show() {
  modulinoPixels.clear();
  modulinoPixels.show();
}

static void _matrix_draw_setup(int b) {
  // Center block 3x3
  for (int y = 2; y <= 4; y++) {
    for (int x = 5; x <= 7; x++) {
      _matrix_set_xy(x, y, BLUE, b);
    }
  }
}

static void _matrix_draw_detect(int b) {
  // Two eyes
  _matrix_set_xy(4, 2, BLUE, b);
  _matrix_set_xy(8, 2, BLUE, b);
}

static void _matrix_draw_grab(int b) {
  // Red horizontal bar
  for (int x = 2; x <= 10; x++) {
    _matrix_set_xy(x, 3, RED, b);
    _matrix_set_xy(x, 4, RED, b);
  }
}

static void _matrix_draw_release(int b) {
  // Green X
  for (int i = 0; i < 8; i++) {
    int x1 = 2 + i;
    int x2 = 10 - i;
    if (x1 >= 0 && x1 < MATRIX_W) _matrix_set_xy(x1, i, GREEN, b);
    if (x2 >= 0 && x2 < MATRIX_W) _matrix_set_xy(x2, i, GREEN, b);
  }
}

int led_matrix_set_intensity(int level) {
#if HAS_MODULINO_CORE
  if (level < 0) level = 0;
  if (level > 15) level = 15;
  led_matrix_intensity = level;
  return 1;
#else
  (void)level;
  return -1;
#endif
}

int led_matrix_clear() {
#if HAS_MODULINO_CORE
  if (!modulino_pixels_ready) return -1;
  _matrix_clear_show();
  return 1;
#else
  return -1;
#endif
}

int led_matrix_set_state(int state_code) {
#if HAS_MODULINO_CORE
  if (!modulino_pixels_ready) return -1;
  int b = _pixels_brightness();
  modulinoPixels.clear();

  // 0=detect, 1=grab, 2=release, 3=setup
  if (state_code == 0) {
    _matrix_draw_detect(b);
    modulinoPixels.show();
    return 1;
  }
  if (state_code == 1) {
    _matrix_draw_grab(b);
    modulinoPixels.show();
    return 1;
  }
  if (state_code == 2) {
    _matrix_draw_release(b);
    modulinoPixels.show();
    return 1;
  }
  if (state_code == 3) {
    _matrix_draw_setup(b);
    modulinoPixels.show();
    return 1;
  }
  modulinoPixels.show();
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

#if HAS_MODULINO_CORE
  Wire.begin();
  Modulino.begin();
  delay(50);
  modulinoPixels.begin();
  modulino_pixels_ready = true;
  led_matrix_set_state(3);
#endif

  pinMode(PRESSURE_PIN_A0, INPUT);
  pinMode(PRESSURE_PIN_A1, INPUT);

  Bridge.provide_safe("move_servo", move_servo);
  Bridge.provide_safe("gripper_hold", gripper_hold);
  Bridge.provide_safe("read_pressure_a0", read_pressure_a0);
  Bridge.provide_safe("read_pressure_a1", read_pressure_a1);
  Bridge.provide_safe("sts_ping", sts_ping);
  Bridge.provide_safe("sts_read_pos", sts_read_pos);
  Bridge.provide_safe("sts_move_pos", sts_move_pos);
  Bridge.provide_safe("sts_torque_enable", sts_torque_enable);
  Bridge.provide_safe("led_matrix_available", led_matrix_available);
  Bridge.provide_safe("led_matrix_set_intensity", led_matrix_set_intensity);
  Bridge.provide_safe("led_matrix_clear", led_matrix_clear);
  Bridge.provide_safe("led_matrix_set_state", led_matrix_set_state);
}

void loop() {}
