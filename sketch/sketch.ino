#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <Servo.h>
#include <SCServo.h>
#include <Arduino_LED_Matrix.h>
#if __has_include(<Modulino.h>)
#include <Modulino.h>
#define HAS_MODULINO_CORE 1
#elif __has_include(<Arduino_Modulino.h>)
#include <Arduino_Modulino.h>
#define HAS_MODULINO_CORE 1
#else
#define HAS_MODULINO_CORE 0
#endif

#define SERVO_PIN 9
#define PRESSURE_PIN_A0 A0
#define PRESSURE_PIN_A1 A1

Servo servo;
SMS_STS sts;  // STS3215 serial bus controller

Arduino_LED_Matrix coreMatrix;
bool core_matrix_ready = false;

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
  return (modulino_pixels_ready || core_matrix_ready) ? 1 : 0;
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

static void _linear_draw_setup(int b) {
  for (int i = 0; i < 8; i += 2) modulinoPixels.set(i, BLUE, b);
}

static void _linear_draw_detect(int b) {
  modulinoPixels.set(2, BLUE, b);
  modulinoPixels.set(5, BLUE, b);
}

static void _linear_draw_grab(int b) {
  for (int i = 0; i < 8; i++) modulinoPixels.set(i, RED, b);
}

static void _linear_draw_release(int b) {
  for (int i = 0; i < 8; i++) modulinoPixels.set(i, GREEN, b);
}

// ---- On-board UNO Q 8x13 LED matrix (Arduino_LED_Matrix) ----

// Simple letter shapes (S/D/G/R) + setup step numbers 1/2/3.

// S1
uint8_t CORE_SETUP1_LOGO[104] = {
  0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,1,1,1,0,0,0,0,0,0,0,0,
  0,0,1,0,0,0,0,0,1,0,0,0,0,
  0,0,1,1,1,0,0,0,1,0,0,0,0,
  0,0,0,0,1,0,0,0,1,0,0,0,0,
  0,0,1,1,1,0,0,0,1,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0
};

// S2
uint8_t CORE_SETUP2_LOGO[104] = {
  0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,1,1,1,0,0,0,0,0,0,0,0,
  0,0,1,0,0,0,0,0,1,0,0,0,0,
  0,0,1,1,1,0,0,0,1,1,0,0,0,
  0,0,1,0,0,0,0,0,1,0,1,0,0,
  0,0,1,1,1,0,0,0,1,1,1,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0
};

// S3
uint8_t CORE_SETUP3_LOGO[104] = {
  0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,1,1,1,0,0,0,0,0,0,0,0,
  0,0,1,0,0,0,0,0,1,0,1,0,0,
  0,0,1,1,1,0,0,0,0,1,0,0,0,
  0,0,1,0,0,0,0,0,0,1,0,0,0,
  0,0,1,1,1,0,0,0,1,0,1,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0
};

// D
uint8_t CORE_DETECT_LOGO[104] = {
  0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,1,1,0,0,0,0,0,0,0,0,0,
  0,0,1,0,1,0,0,0,0,0,0,0,0,
  0,0,1,0,0,1,0,0,0,0,0,0,0,
  0,0,1,0,0,1,0,0,0,0,0,0,0,
  0,0,1,0,1,0,0,0,0,0,0,0,0,
  0,0,1,1,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0
};

// G
uint8_t CORE_GRAB_LOGO[104] = {
  0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,1,1,0,0,0,0,0,0,0,0,
  0,0,1,0,0,1,0,0,0,0,0,0,0,
  0,0,1,0,0,0,0,0,0,0,0,0,0,
  0,0,1,0,1,1,0,0,0,0,0,0,0,
  0,0,1,0,0,1,0,0,0,0,0,0,0,
  0,0,0,1,1,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0
};

// R
uint8_t CORE_RELEASE_LOGO[104] = {
  0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,1,1,1,0,0,0,0,0,0,0,0,
  0,0,1,0,0,1,0,0,0,0,0,0,0,
  0,0,1,0,0,1,0,0,0,0,0,0,0,
  0,0,1,1,1,0,0,0,0,0,0,0,0,
  0,0,1,0,1,0,0,0,0,0,0,0,0,
  0,0,1,0,0,1,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0
};

// Error base pattern "ER" on the left, room for small digit on the right.
uint8_t CORE_ERROR1_LOGO[104] = {
  // ER1
  0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,1,1,1,0,0,1,0,0,0,0,1,0,
  0,1,0,0,0,0,1,0,0,0,1,1,0,
  0,1,1,1,0,0,1,0,0,0,0,1,0,
  0,1,0,0,0,0,1,0,0,0,0,1,0,
  0,1,1,1,0,0,1,0,0,0,0,1,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0
};

uint8_t CORE_ERROR2_LOGO[104] = {
  // ER2
  0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,1,1,1,0,0,1,0,0,0,1,1,0,
  0,1,0,0,0,0,1,0,0,1,0,1,0,
  0,1,1,1,0,0,1,0,0,0,1,1,0,
  0,1,0,0,0,0,1,0,0,1,0,0,0,
  0,1,1,1,0,0,1,0,0,1,1,1,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0
};

uint8_t CORE_ERROR3_LOGO[104] = {
  // ER3
  0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,1,1,1,0,0,1,0,0,0,1,1,0,
  0,1,0,0,0,0,1,0,0,0,0,1,0,
  0,1,1,1,0,0,1,0,0,0,1,1,0,
  0,1,0,0,0,0,1,0,0,0,0,1,0,
  0,1,1,1,0,0,1,0,0,0,1,1,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0
};

uint8_t CORE_ERROR4_LOGO[104] = {
  // ER4
  0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,1,1,1,0,0,1,0,0,1,0,1,0,
  0,1,0,0,0,0,1,0,0,1,0,1,0,
  0,1,1,1,0,0,1,0,0,1,1,1,0,
  0,1,0,0,0,0,1,0,0,0,0,1,0,
  0,1,1,1,0,0,1,0,0,0,0,1,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0
};

uint8_t CORE_ERROR5_LOGO[104] = {
  // ER5
  0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,1,1,1,0,0,1,0,0,1,1,1,0,
  0,1,0,0,0,0,1,0,0,1,0,0,0,
  0,1,1,1,0,0,1,0,0,1,1,1,0,
  0,1,0,0,0,0,1,0,0,0,0,1,0,
  0,1,1,1,0,0,1,0,0,1,1,1,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0
};

void _core_matrix_set_state(int state_code) {
  if (!core_matrix_ready) return;
  if (state_code == 0) {
    coreMatrix.draw(CORE_DETECT_LOGO);
    return;
  }
  if (state_code == 1) {
    coreMatrix.draw(CORE_GRAB_LOGO);
    return;
  }
  if (state_code == 2) {
    coreMatrix.draw(CORE_RELEASE_LOGO);
    return;
  }
  if (state_code == 3) {
    // Generic setup (if phase not specified).
    coreMatrix.draw(CORE_SETUP1_LOGO);
    return;
  }
}

int core_matrix_set_setup_step(int step) {
  if (!core_matrix_ready) return -1;
  if (step == 1) {
    coreMatrix.draw(CORE_SETUP1_LOGO);
    return 1;
  }
  if (step == 2) {
    coreMatrix.draw(CORE_SETUP2_LOGO);
    return 1;
  }
  if (step == 3) {
    coreMatrix.draw(CORE_SETUP3_LOGO);
    return 1;
  }
  return 0;
}

int core_matrix_set_error_code(int code) {
  if (!core_matrix_ready) return -1;
  if (code == 1) {
    coreMatrix.draw(CORE_ERROR1_LOGO);
    return 1;
  }
  if (code == 2) {
    coreMatrix.draw(CORE_ERROR2_LOGO);
    return 1;
  }
  if (code == 3) {
    coreMatrix.draw(CORE_ERROR3_LOGO);
    return 1;
  }
  if (code == 4) {
    coreMatrix.draw(CORE_ERROR4_LOGO);
    return 1;
  }
  if (code == 5) {
    coreMatrix.draw(CORE_ERROR5_LOGO);
    return 1;
  }
  return 0;
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
  // Always update on-board matrix if available.
  _core_matrix_set_state(state_code);

#if HAS_MODULINO_CORE
  if (modulino_pixels_ready) {
    int b = _pixels_brightness();
    modulinoPixels.clear();

    // 0=detect, 1=grab, 2=release, 3=setup
    if (state_code == 0) {
      _linear_draw_detect(b);   // fallback for 8-led mapping
      _matrix_draw_detect(b);
      modulinoPixels.show();
      return 1;
    }
    if (state_code == 1) {
      _linear_draw_grab(b);     // fallback for 8-led mapping
      _matrix_draw_grab(b);
      modulinoPixels.show();
      return 1;
    }
    if (state_code == 2) {
      _linear_draw_release(b);  // fallback for 8-led mapping
      _matrix_draw_release(b);
      modulinoPixels.show();
      return 1;
    }
    if (state_code == 3) {
      _linear_draw_setup(b);    // fallback for 8-led mapping
      _matrix_draw_setup(b);
      modulinoPixels.show();
      return 1;
    }
    modulinoPixels.show();
  }
  return 0;
#else
  (void)state_code;
  return core_matrix_ready ? 1 : -1;
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

  // On-board LED matrix
  coreMatrix.begin();
  coreMatrix.setGrayscaleBits(1);
  core_matrix_ready = true;
  _core_matrix_set_state(3);  // setup state at boot
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
  Bridge.provide_safe("core_matrix_set_setup_step", core_matrix_set_setup_step);
  Bridge.provide_safe("core_matrix_set_error_code", core_matrix_set_error_code);
}

void loop() {}
