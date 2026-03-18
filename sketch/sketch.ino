#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <Servo.h>
#include <SCServo.h>
#include <Arduino_LED_Matrix.h>
#ifdef endTextAnimation
#undef endTextAnimation
#endif
#ifdef beginTextAnimation
#undef beginTextAnimation
#endif
#include <ArduinoGraphics.h>
#include "Modulino_LED_Matrix.h"
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

int led_matrix_intensity = 3;  // 0..15, API compatibility

// Head matrix (Modulino LED Matrix). Logical coordinates: x=0..11, y=0..7.
ModulinoLEDMatrix headMatrix;
bool head_matrix_ready = false;
volatile int head_state_code = 3; // 0=detect,1=grab,2=release,3=setup
unsigned long head_last_blink_ms = 0;
bool head_blink_closed = false;
unsigned long head_last_tick_ms = 0;
int head_last_render_state = -1;
bool head_last_render_blink = false;

static inline int _clamp_i(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Head matrix orientation mapping.
static void _head_point(int x, int y) {
  // No rotation (direct mapping). If still rotated, we can switch to 90° mapping.
  x = _clamp_i(x, 0, 11);
  y = _clamp_i(y, 0, 7);
  headMatrix.point(x, y);
}

static void _head_rect(int x, int y, int w, int h) {
  for (int yy = y; yy < (y + h); yy++) {
    for (int xx = x; xx < (x + w); xx++) {
      _head_point(xx, yy);
    }
  }
}

static void _head_clear_draw() {
  headMatrix.clear();
  headMatrix.beginDraw();
  headMatrix.stroke(0xFFFFFF);
  headMatrix.fill(0xFFFFFF);
}

static void _head_end_draw() { headMatrix.endDraw(); }

static void _head_capsule_filled(int x, int y, int w, int h) {
  // Draw a filled vertical capsule using pixel math (rect + 2 semicircles).
  if (w < 1 || h < 1) return;
  int r = w / 2;
  if (r < 1) r = 1;
  int cx = x + (w - 1) / 2;
  int cy_top = y + r;
  int cy_bot = y + h - 1 - r;
  for (int yy = y; yy < (y + h); yy++) {
    for (int xx = x; xx < (x + w); xx++) {
      bool inside = false;
      if (yy >= (y + r) && yy <= (y + h - 1 - r)) {
        inside = true; // middle rectangle
      } else if (yy < (y + r)) {
        int dx = xx - cx;
        int dy = yy - cy_top;
        inside = (dx * dx + dy * dy) <= (r * r);
      } else {
        int dx = xx - cx;
        int dy = yy - cy_bot;
        inside = (dx * dx + dy * dy) <= (r * r);
      }
      if (inside) _head_point(xx, yy);
    }
  }
}

static void _head_draw_eyes_open() {
  // Vertical capsule eyes (filled)
  _head_capsule_filled(2, 1, 3, 6);
  _head_capsule_filled(7, 1, 3, 6);
}

static void _head_draw_eyes_blink() {
  // Thin blink line
  _head_rect(2, 4, 3, 1);
  _head_rect(7, 4, 3, 1);
}

static void _head_draw_eyes_grab() {
  // Focused eyes: same capsules
  _head_capsule_filled(2, 1, 3, 6);
  _head_capsule_filled(7, 1, 3, 6);
}

static void _head_draw_eyes_release() {
  // Happy eyes: bigger capsules
  _head_capsule_filled(1, 1, 4, 6);
  _head_capsule_filled(7, 1, 4, 6);
}

static void _head_render_state(int state_code, bool blink_closed) {
  if (!head_matrix_ready) return;
  _head_clear_draw();
  if (state_code == 0) { // detect
    if (blink_closed) _head_draw_eyes_blink();
    else _head_draw_eyes_open();
  } else if (state_code == 1) { // grab
    _head_draw_eyes_grab();
  } else if (state_code == 2) { // release
    _head_draw_eyes_release();
  } else { // setup
    _head_draw_eyes_open();
    // No extra indicator pixels (avoid "stuck" top-right LED)
  }
  _head_end_draw();
}

static void _head_tick() {
  if (!head_matrix_ready) return;
  unsigned long now = millis();
  if (now - head_last_tick_ms < 60) return;
  head_last_tick_ms = now;

  if (head_state_code == 0 || head_state_code == 3) {
    if (now - head_last_blink_ms > 3500) {
      head_blink_closed = true;
      head_last_blink_ms = now;
    }
    if (head_blink_closed && (now - head_last_blink_ms > 160)) {
      head_blink_closed = false;
    }
    if (head_last_render_state != head_state_code || head_last_render_blink != head_blink_closed) {
      _head_render_state(head_state_code, head_blink_closed);
      head_last_render_state = head_state_code;
      head_last_render_blink = head_blink_closed;
    }
  }
}

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

int led_matrix_available() {
  return (head_matrix_ready || core_matrix_ready) ? 1 : 0;
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
  // Head matrix (ModulinoLEDMatrix) does not expose a brightness API in this project.
  // Intensity for the UNO Q onboard coreMatrix is not supported by Arduino_LED_Matrix
  // in this environment, so we only keep the RAM value for API compatibility.
  if (level < 0) level = 0;
  if (level > 15) level = 15;
  led_matrix_intensity = level;
  return 1;
}

int led_matrix_clear() {
  if (!head_matrix_ready) return -1;
  headMatrix.clear();
  return 1;
}

int led_matrix_set_state(int state_code) {
  // Always update on-board matrix if available.
  _core_matrix_set_state(state_code);

  head_state_code = state_code;
  _head_render_state(state_code, false);
  head_last_render_state = state_code;
  head_last_render_blink = false;
  return (head_matrix_ready || core_matrix_ready) ? 1 : -1;
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

  // Head LED matrix (Modulino LED Matrix)
#if HAS_MODULINO_CORE
  Modulino.begin();
#endif
  delay(50);
  if (headMatrix.begin()) {
    head_matrix_ready = true;
    _head_render_state(3, false);
  } else {
    head_matrix_ready = false;
  }

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

void loop() {
  _head_tick();
  // Yield to let RouterBridge and other tasks run reliably on Zephyr.
  delay(2);
}
