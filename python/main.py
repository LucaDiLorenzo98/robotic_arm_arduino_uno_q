# SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL (http://www.arduino.cc)
#
# SPDX-License-Identifier: MPL-2.0

from arduino.app_utils import App, Bridge
from arduino.app_bricks.video_objectdetection import VideoObjectDetection
from datetime import datetime, UTC
import time
import threading
import math
import os
import sys
from functools import partial


def _enable_unbuffered_console_and_file_log() -> None:
    """
    Best-effort: make prints visible in real time.
    Some launchers capture/buffer stdout; this forces flushing and also mirrors
    stdout/stderr to a local log file for debugging.
    """

    # 1) Force line-buffering (when supported) so lines appear immediately.
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    try:
        sys.stderr.reconfigure(line_buffering=True)
    except Exception:
        pass

    # 2) Mirror stdout/stderr to a file in this project (useful when console is muted).
    log_path = os.environ.get("ROBOT_LOG_PATH") or os.path.join(
        os.path.dirname(__file__), "robot.log"
    )
    try:
        log_f = open(log_path, "a", encoding="utf-8", buffering=1)
    except Exception:
        return

    class _Tee:
        def __init__(self, a, b):
            self._a = a
            self._b = b
            self._at_line_start = True

        def write(self, s):
            if not s:
                return 0
            # Prepend timestamp at the start of each line.
            from datetime import datetime as _dt
            parts = s.split("\n")
            out_parts = []
            for i, part in enumerate(parts):
                if i > 0:
                    out_parts.append("\n")
                    self._at_line_start = True
                if part:
                    if self._at_line_start:
                        ts = _dt.now().strftime("%H:%M:%S.%f")[:-3]
                        out_parts.append(f"[{ts}] {part}")
                    else:
                        out_parts.append(part)
                    self._at_line_start = False
            stamped = "".join(out_parts)
            try:
                self._a.write(stamped)
            except Exception:
                pass
            try:
                self._b.write(stamped)
            except Exception:
                pass
            return len(s)

        def flush(self):
            try:
                self._a.flush()
            except Exception:
                pass
            try:
                self._b.flush()
            except Exception:
                pass

        def isatty(self):
            try:
                return bool(getattr(self._a, "isatty")())
            except Exception:
                return False

    sys.stdout = _Tee(sys.stdout, log_f)
    sys.stderr = _Tee(sys.stderr, log_f)


_enable_unbuffered_console_and_file_log()

# Ensure every print flushes by default (helps with buffered stdout).
print = partial(print, flush=True)

# ID dei servo STS3215 (solo 4 presenti sul bus)
STS_SERVO_IDS = (1, 2, 3, 4)
STS_CHECK_IDS = STS_SERVO_IDS
STS_STARTUP_DELAY_SEC = 6.0  # Attesa prima del check (Bridge/MCU/servo bus devono essere pronti)

# Bridge calls can time out during early startup or transient USB/serial hiccups.
BRIDGE_CALL_RETRIES = 3
BRIDGE_CALL_RETRY_DELAY_SEC = 0.25
BRIDGE_RESULT_TIMEOUT_SEC = 5.0  # Prevent deadlock if MCU never responds to .result()



def _print_sts_servo_info():
    """Stampa a console ping e posizione per ogni ID STS3215 (per check)."""
    print("[STS3215] Check servo (ping + posizione):\n")
    all_ok = True
    for sid in STS_CHECK_IDS:
        print(f"[STS3215] checking id={sid} ...")
        ok = sts_ping(sid)
        pos = sts_read_pos(sid)
        pos_str = str(pos) if pos is not None else "—"
        status = "OK" if ok else "NO"
        all_ok = all_ok and ok and pos is not None
        print(f"  ID={sid}  ping={status}  pos={pos_str}")
    print()
    return all_ok


def _startup_sts_check():
    """Esegue il check STS dopo un breve delay (Bridge deve essere connesso)."""
    try:
        time.sleep(STS_STARTUP_DELAY_SEC)
        # Initialize LED matrix now that Bridge is ready.
        try:
            led_matrix_set_intensity(_led_matrix_intensity)
            led_matrix_set_state_name(STATE_SETUP)
        except Exception:
            pass
        # Clear indication: now it's safe to start setup and move the robot by hand.
        core_matrix_set_setup_step(1)
        print("[SETUP] Move robot to HOME, then hold still 5s.")
        print("[SETUP] Then move robot to FIRST_SLOT, then hold still 5s.")
        print("[SETUP] Then move robot to SECOND_SLOT, then hold still 5s.")
        # Ready for setup: nod + open gripper slightly
        _set_sts_torque_enabled(True)
        _setup_nod_servo4()
        _set_sts_torque_enabled(False)
        _set_gripper_setup_open_free()
        all_ok = _print_sts_servo_info()
        if not all_ok:
            core_matrix_set_error_code(ERR_SETUP_POSE_UNAVAILABLE)
            print(
                "[SETUP][ERROR] One or more STS3215 servos did not respond (check wiring/half-duplex adapter/IDs)."
            )
        print("[SETUP] READY: you can move the arm now.")
    except Exception as e:
        print(f"[STS3215] startup check failed: {e}")
    finally:
        _startup_check_done.set()


# State machine: setup -> detect -> (face) -> grab -> delivery -> (face) -> release -> detect
STATE_SETUP = "setup"
STATE_DETECT = "detect"
STATE_GRAB = "grab"
STATE_DELIVERY = "delivery"
STATE_RELEASE = "release"

# Gripper servo angles (adjust if your servo is reversed)
GRIPPER_OPEN_ANGLE = 150  # fully open
GRIPPER_CLOSED_ANGLE = 180  # fully closed (max grip)
GRIPPER_GRAB_PREOPEN_ANGLE = 165  # narrow opening (~2cm) to grab a pen
GRIPPER_GRAB_PRE_CLOSE_ANGLE = 25  # micro-close after reaching gadget point
GRIPPER_GRAB_PRE_CLOSE_WAIT_SEC = 0.35
GRIPPER_SETUP_OPEN_ANGLE = GRIPPER_GRAB_PREOPEN_ANGLE  # same opening as pre-grab

GRAB_OPEN_WAIT_SEC = 1.0
GRAB_CLOSE_WAIT_SEC = 1.0
GRAB_WAIT_BEFORE_HOME_SEC = 15.0
GRAB_SETTLE_BEFORE_CLOSE_SEC = 2.0
GRAB_APPROACH_SPEED = 700
GRAB_APPROACH_ACC = 20
GRAB_RETURN_SPEED = 800
GRAB_RETURN_ACC = 25
RELEASE_OPEN_WAIT_SEC = 3.0
DELIVERY_HEART_SEC = 3.0  # show heart eyes then release pen
DELIVERY_POST_RELEASE_HOME_SEC = 5.0  # wait at home before returning to idle

# Delivery pose (RAM-only). If not set, computed as HOME + DELIVERY_DELTAS (no base rotation).
DELIVERY_POSITION: dict[int, int] = {}
DELIVERY_DELTAS = {2: 120, 3: -120, 4: 0}
DELIVERY_SPEED = 650
DELIVERY_ACC = 18

# Posizione "home" del braccio STS3215 (RAM-only session).
HOME_DEFAULT_POSITION = {
    1: 2048,
    2: 2048,
    3: 2048,
    4: 2048,
}
HOME_POSITION = dict(HOME_DEFAULT_POSITION)
HOME_MOVE_SPEED = 900
HOME_MOVE_ACC = 30
HOME_SETTLE_SEC = 1.2

# Primo slot (RAM-only session)
FIRST_SLOT_POSITION: dict[int, int] = {}
# Secondo slot (RAM-only session): usato per calcolare offset tra slot.
SECOND_SLOT_POSITION: dict[int, int] = {}

# Setup automatico: fermo per N secondi -> salva posizione.
SETUP_STABLE_SECONDS = 5.0
# Threshold for detecting "manual movement" while torque is OFF.
# Too low can trigger on micro-movement / sensor noise during settling.
SETUP_MOVEMENT_THRESHOLD = 40

# After switching torque OFF, ignore movement detection for a short grace period
# so the arm can settle under gravity without automatically advancing setup.
SETUP_MOVEMENT_ARM_DELAY_SEC = 1.5
SETUP_NOD_DELTA_SERVO4 = 120
SETUP_NOD_SPEED = 500
SETUP_NOD_ACC = 20
SETUP_NOD_DELAY_SEC = 0.30
SETUP_BLINK_INTERVAL_SEC = 0.5  # blink period during "hold still" countdown

# Grab sequenziale su 9 slot con offset base servo1 configurabile.
GRAB_SLOT_COUNT_MAX = 9
GRAB_BASE_SERVO_ID = 1
GRAB_BASE_OFFSET_STEP = 300
GRAB_BASE_OFFSET_DIRECTION = -1  # +1 o -1

# Web UI disabilitata: manteniamo solo face detection + logica robot.
ui = None
detection_stream = VideoObjectDetection(confidence=0.85, debounce_sec=0.0)

_state = STATE_SETUP
_state_lock = threading.Lock()
_setup_phase = "home"  # home -> first_slot -> second_slot -> done
_setup_prev_pose: dict[int, int] | None = None
_setup_stable_since = 0.0
_setup_phase_moved = False  # diventa True quando rileva movimento manuale nella fase corrente
_setup_anchor_pose: dict[int, int] | None = None  # pose recorded when arm first appears still
_setup_torque_free_active = False
_setup_pose_fail_streak = 0
_setup_last_blink_ts = 0.0
_setup_blink_on = True
_slot_grab_count = 0
_grab_offset_warn_ts = 0.0
_setup_movement_arm_ts = 0.0

# Cooldown: after setup→detect, ignore face detections for a few seconds
# so the system stabilises (idle animation init, Bridge settling, etc.).
DETECT_COOLDOWN_SEC = 4.0
_detect_cooldown_until = 0.0
_delivery_face_seen = threading.Event()  # signaled when face detected during delivery

# Wait until the startup STS/LED routine has finished (servo nod + ping).
_startup_check_done = threading.Event()
_pose_capture_fail_print_ts = 0.0

# Idle animation (STS3215) configuration
IDLE_ANIM_ENABLED = True
IDLE_ANIM_PERIOD_SEC = 10.0  # più dinamico ma ancora naturale
IDLE_ANIM_CHECK_INTERVAL_SEC = 0.015
IDLE_ANIM_AMPLITUDES = {
    1: 210,
    2: 170,
    3: 140,
    4: 110,
}
IDLE_ANIM_SPEED = 420
IDLE_ANIM_ACC = 12
IDLE_ANIM_MAX_STEP = 3   # passo ancora più piccolo = più fluido
IDLE_ANIM_DEADBAND = 1   # aggiorna quasi continuo
IDLE_ANIM_MIN_CMD_INTERVAL_SEC = 0.02
IDLE_ANIM_FILTER_ALPHA = 0.08
IDLE_ANIM_PAUSE_AFTER_MANUAL_SEC = 5.0
IDLE_ANIM_ACTIVE_WINDOW_SEC = 10.0
IDLE_ANIM_REST_WINDOW_SEC = 30.0

# Push realtime UI telemetry (socket events)
TELEMETRY_READER_INTERVAL_SEC = 0.10
TELEMETRY_PUSH_INTERVAL_SEC = 0.10
TELEMETRY_STATE_HEARTBEAT_SEC = 2.0
TELEMETRY_LOG_INTERVAL_SEC = 2.0
LED_MATRIX_DEFAULT_INTENSITY = 3

# Error codes for core matrix 'ERx' display.
ERR_SETUP_POSE_UNAVAILABLE = 1
ERR_GRAB_SLOT_NOT_CONFIGURED = 2
ERR_STS_MOVE_FAILED = 3
ERR_MATRIX_UNAVAILABLE = 4  # deprecated (no longer shown at boot)
ERR_PRESSURE_READ_FAILED = 5

_idle_base_pos: dict[int, int] = {}
_idle_traj_state: dict[int, dict[str, float]] = {}
_idle_lock = threading.Lock()
_idle_pause_until = 0.0  # timestamp: finché > now, animazione sospesa (es. dopo comando manuale)

# Bridge calls are not thread-safe: serialize all calls with a single lock.
_bridge_lock = threading.Lock()
_led_matrix_intensity = LED_MATRIX_DEFAULT_INTENSITY
def _validated_home_position(raw) -> dict[int, int]:
    """Validate and normalize home dict from JSON-like data."""
    out: dict[int, int] = {}
    if not isinstance(raw, dict):
        return {}
    for sid in STS_SERVO_IDS:
        val = raw.get(str(sid), raw.get(sid))
        if val is None:
            return {}
        try:
            v = int(val)
        except Exception:
            return {}
        if not 0 <= v <= 4095:
            return {}
        out[sid] = v
    return out


def _get_request_arg(name: str, fallback=None):
    """Read query-string arg safely (works even when flask request is unavailable)."""
    if fallback is not None:
        return fallback
    try:
        from flask import request
        return request.args.get(name)
    except Exception:
        return None


def _capture_current_pose() -> dict[int, int] | None:
    """Read current pose for all configured STS servos and validate it."""
    # STS reads can fail transiently (return None or negative). Retry briefly
    # to avoid aborting setup on a single bad read.
    for _ in range(3):
        current: dict[int, int] = {}
        ok = True
        for sid in STS_SERVO_IDS:
            pos = sts_read_pos(sid)
            if pos is None:
                global _pose_capture_fail_print_ts
                now_ts = time.time()
                # Rate limit: print at most once per ~2s.
                if now_ts - _pose_capture_fail_print_ts > 2.0:
                    print(f"[STS3215] capture pose failed at id={sid}.")
                    _pose_capture_fail_print_ts = now_ts
                ok = False
                break
            current[sid] = int(pos)
        if ok:
            valid = _validated_home_position(current)
            if valid:
                return valid
        time.sleep(0.03)
    return None


def _pose_payload(pose: dict[int, int]) -> dict[str, int]:
    return {str(k): int(v) for k, v in pose.items()}


def _pause_idle_temporarily():
    global _idle_pause_until
    _idle_pause_until = time.time() + IDLE_ANIM_PAUSE_AFTER_MANUAL_SEC


def _set_sts_torque_enabled(enabled: bool) -> bool:
    """Enable/disable torque for all STS servos."""
    ok_all = True
    for sid in STS_SERVO_IDS:
        ok = sts_set_torque(sid, enabled)
        ok_all = ok_all and ok
    mode = "ON" if enabled else "OFF"
    print(f"[STS3215] torque {mode} (all): {ok_all}")
    return ok_all


def _set_home_from_current_pose():
    """Set HOME_POSITION from current servo pose at startup (session RAM home)."""
    global HOME_POSITION
    valid = _capture_current_pose()
    if not valid:
        return False
    HOME_POSITION = dict(valid)
    print(f"[HOME] Startup home set from current pose: {HOME_POSITION}")
    return True


def _read_current_pose() -> dict[int, int] | None:
    return _capture_current_pose()


def _pose_has_movement(pose_a: dict[int, int], pose_b: dict[int, int], threshold: int) -> bool:
    for sid in STS_SERVO_IDS:
        if abs(int(pose_a.get(sid, 0)) - int(pose_b.get(sid, 0))) > threshold:
            return True
    return False


def _setup_current_step_number() -> int:
    """Map setup phase to display step: S1=HOME, S2=FIRST_SLOT, S3=SECOND_SLOT."""
    if _setup_phase == "home":
        return 1
    if _setup_phase == "first_slot":
        return 2
    if _setup_phase == "second_slot":
        return 3
    return 1


def _setup_nod_servo4():
    sid = 4
    center = HOME_POSITION.get(sid)
    if center is None:
        return
    left = max(0, min(4095, int(center - SETUP_NOD_DELTA_SERVO4)))
    right = max(0, min(4095, int(center + SETUP_NOD_DELTA_SERVO4)))
    sts_move_pos(sid, right, speed=SETUP_NOD_SPEED, acc=SETUP_NOD_ACC)
    time.sleep(SETUP_NOD_DELAY_SEC)
    sts_move_pos(sid, left, speed=SETUP_NOD_SPEED, acc=SETUP_NOD_ACC)
    time.sleep(SETUP_NOD_DELAY_SEC)
    sts_move_pos(sid, int(center), speed=SETUP_NOD_SPEED, acc=SETUP_NOD_ACC)
    time.sleep(SETUP_NOD_DELAY_SEC)


def _handle_setup_phase():
    """Auto setup: hold still 5s to capture HOME, FIRST_SLOT, SECOND_SLOT."""
    global _setup_prev_pose, _setup_stable_since, _setup_phase, _setup_phase_moved, _setup_torque_free_active
    global _setup_pose_fail_streak, _setup_last_blink_ts, _setup_blink_on
    global _setup_movement_arm_ts
    global HOME_POSITION, FIRST_SLOT_POSITION, SECOND_SLOT_POSITION, _slot_grab_count
    global GRAB_BASE_OFFSET_STEP, GRAB_BASE_OFFSET_DIRECTION
    now = time.time()
    if not _setup_torque_free_active:
        _set_sts_torque_enabled(False)
        _setup_torque_free_active = True
        _setup_movement_arm_ts = now + SETUP_MOVEMENT_ARM_DELAY_SEC
    pose = _read_current_pose()
    if pose is None:
        _setup_pose_fail_streak += 1
        if _setup_pose_fail_streak == 1:
            print("[SETUP] pose read returned None (STS timeout or invalid read).")
        # Only show error after a short streak, otherwise keep trying silently.
        if _setup_pose_fail_streak >= 10:
            core_matrix_set_error_code(ERR_SETUP_POSE_UNAVAILABLE)
        return
    _setup_pose_fail_streak = 0

    if _setup_prev_pose is None:
        _setup_prev_pose = dict(pose)
        _setup_stable_since = now
        print(f"[SETUP] Phase={_setup_phase}: move arm by hand, then keep still {SETUP_STABLE_SECONDS:.0f}s.")
        return

    # Ignore movement detection during the first delay after torque OFF,
    # otherwise the arm "settling" can auto-advance setup.
    if now < _setup_movement_arm_ts:
        _setup_prev_pose = dict(pose)
        return

    global _setup_anchor_pose
    moved = _pose_has_movement(pose, _setup_prev_pose, SETUP_MOVEMENT_THRESHOLD)
    _setup_prev_pose = dict(pose)
    if moved:
        if not _setup_phase_moved:
            print(f"[SETUP] Phase={_setup_phase}: movement detected, waiting stable hold...")
        _setup_phase_moved = True
        _setup_stable_since = now
        _setup_anchor_pose = None  # reset anchor on any fast movement
        return

    # Evita catture premature: la fase si arma solo dopo almeno un movimento manuale.
    if not _setup_phase_moved:
        return

    # Compare against anchor pose (catches slow drift during repositioning).
    # The anchor is set when the arm first appears still after movement.
    if _setup_anchor_pose is None:
        _setup_anchor_pose = dict(pose)
        _setup_stable_since = now  # timer starts from anchor
    else:
        drifted = _pose_has_movement(pose, _setup_anchor_pose, SETUP_MOVEMENT_THRESHOLD)
        if drifted:
            # Slow drift from anchor — arm is still being repositioned
            _setup_anchor_pose = dict(pose)
            _setup_stable_since = now
            return

    if (now - _setup_stable_since) < SETUP_STABLE_SECONDS:
        # Blink the step number on the onboard matrix so the user sees "hold still".
        if (now - _setup_last_blink_ts) >= SETUP_BLINK_INTERVAL_SEC:
            _setup_last_blink_ts = now
            _setup_blink_on = not _setup_blink_on
            step = _setup_current_step_number()
            if _setup_blink_on:
                core_matrix_set_setup_step(step)
            else:
                core_matrix_clear()
        return

    if _setup_phase == "home":
        HOME_POSITION = dict(pose)
        print(f"[SETUP] HOME captured: {HOME_POSITION}")
        _set_sts_torque_enabled(True)
        _setup_torque_free_active = False
        _setup_nod_servo4()
        _set_sts_torque_enabled(False)
        _setup_torque_free_active = True
        _setup_movement_arm_ts = time.time() + SETUP_MOVEMENT_ARM_DELAY_SEC
        _setup_phase = "first_slot"
        _setup_phase_moved = False
        _setup_anchor_pose = None
        _setup_prev_pose = None
        _setup_stable_since = now
        _setup_blink_on = True
        core_matrix_set_setup_step(2)  # S2 = move to first slot
        return

    if _setup_phase == "first_slot":
        FIRST_SLOT_POSITION = dict(pose)
        print(f"[SETUP] FIRST_SLOT captured: {FIRST_SLOT_POSITION}")

        # Return to HOME between slot captures so the user can position
        # the arm consistently for SECOND_SLOT.
        _set_sts_torque_enabled(True)
        _setup_torque_free_active = False
        _move_arm_then_base(
            HOME_POSITION,
            speed=HOME_MOVE_SPEED,
            acc=HOME_MOVE_ACC,
            lift_pose=HOME_POSITION,
        )
        _set_sts_torque_enabled(False)
        _setup_torque_free_active = True
        _set_gripper_setup_open_free()

        _setup_phase = "second_slot"
        _setup_phase_moved = False
        _setup_anchor_pose = None
        _setup_prev_pose = None
        _setup_stable_since = time.time()
        _setup_movement_arm_ts = time.time() + SETUP_MOVEMENT_ARM_DELAY_SEC
        _setup_blink_on = True
        core_matrix_set_setup_step(3)  # S3 = move to second slot
        return

    if _setup_phase == "second_slot":
        SECOND_SLOT_POSITION = dict(pose)
        print(f"[SETUP] SECOND_SLOT captured: {SECOND_SLOT_POSITION}")

        # Auto-calc offset from FIRST_SLOT -> SECOND_SLOT on base servo.
        first_base = int(FIRST_SLOT_POSITION.get(GRAB_BASE_SERVO_ID, 0))
        second_base = int(SECOND_SLOT_POSITION.get(GRAB_BASE_SERVO_ID, first_base))
        delta = second_base - first_base
        if delta != 0:
            GRAB_BASE_OFFSET_DIRECTION = 1 if delta > 0 else -1
            GRAB_BASE_OFFSET_STEP = abs(delta)
        print(
            "[SETUP] slot offset updated: "
            f"step={GRAB_BASE_OFFSET_STEP}, direction={GRAB_BASE_OFFSET_DIRECTION}, delta={delta}"
        )

        _slot_grab_count = 0
        _setup_phase = "done"
        _setup_phase_moved = False
        _setup_anchor_pose = None
        _set_sts_torque_enabled(True)
        _setup_torque_free_active = False
        print("[SETUP] Returning to HOME before idle.")
        # Return rule: lift arm first, then rotate base.
        _move_arm_then_base(HOME_POSITION, speed=HOME_MOVE_SPEED, acc=HOME_MOVE_ACC, lift_pose=HOME_POSITION)
        core_matrix_set_setup_step(3)
        _set_state(STATE_DETECT)


def _set_state(new_state: str):
    global _state, _setup_torque_free_active, _detect_cooldown_until
    old_state = _state
    if old_state == STATE_SETUP and new_state != STATE_SETUP and _setup_torque_free_active:
        _set_sts_torque_enabled(True)
        _setup_torque_free_active = False
    with _state_lock:
        _state = new_state
    if new_state == STATE_DETECT:
        _detect_cooldown_until = time.time() + DETECT_COOLDOWN_SEC
    try:
        led_matrix_set_state_name(new_state)
    except Exception:
        pass
    print(f"[StateMachine] -> {new_state}")


def _move_servo(angle: int):
    try:
        res = _bridge_call_and_unwrap("move_servo", int(angle))
        if res is None:
            print(f"[StateMachine] move_servo({angle}): Bridge returned None")
    except Exception as e:
        print(f"[StateMachine] move_servo({angle}) error: {e}")


def _set_gripper_hold(hold: bool) -> bool:
    try:
        res = _bridge_call_and_unwrap("gripper_hold", 1 if hold else 0)
        return res is not None and int(res) > 0
    except Exception as e:
        print(f"[StateMachine] gripper_hold({hold}) error: {e}")
        return False


def _set_gripper_idle_closed_free():
    # Idle policy: close gripper and hold.
    _set_gripper_hold(True)
    _move_servo(GRIPPER_CLOSED_ANGLE)
    time.sleep(0.2)


def _set_gripper_setup_open_free():
    _set_gripper_hold(True)
    _move_servo(GRIPPER_SETUP_OPEN_ANGLE)
    time.sleep(0.2)
    # Keep holding throughout setup — grip is released when entering detect (idle worker)


def _clamp_pos(v: int) -> int:
    return max(0, min(4095, int(v)))


def _get_delivery_pose() -> dict[int, int]:
    if DELIVERY_POSITION:
        return dict(DELIVERY_POSITION)
    pose = dict(HOME_POSITION)
    for sid, delta in DELIVERY_DELTAS.items():
        pose[sid] = _clamp_pos(int(pose.get(sid, 2048)) + int(delta))
    # Do NOT rotate base for delivery
    pose[GRAB_BASE_SERVO_ID] = int(HOME_POSITION.get(GRAB_BASE_SERVO_ID, pose.get(GRAB_BASE_SERVO_ID, 2048)))
    return pose


def _move_arm_to_home_position():
    """Muove tutti i servo STS verso la home prima del ritorno a idle."""
    for sid in STS_SERVO_IDS:
        target = HOME_POSITION.get(sid)
        if target is None:
            continue
        sts_move_pos(sid, int(target), speed=HOME_MOVE_SPEED, acc=HOME_MOVE_ACC)
    time.sleep(HOME_SETTLE_SEC)



def _move_arm_then_base(target_pose: dict[int, int], speed: int, acc: int, lift_pose: dict[int, int] | None = None) -> bool:
    """Return: lift arm joints first (2..), then rotate base (id=1)."""
    ok_all = True
    # Lift/position arm joints first (keep base unchanged here)
    src = lift_pose if lift_pose is not None else target_pose
    for sid in STS_SERVO_IDS:
        if sid == GRAB_BASE_SERVO_ID:
            continue
        target = src.get(sid)
        if target is None:
            ok_all = False
            continue
        ok_all = ok_all and sts_move_pos(sid, int(target), speed=int(speed), acc=int(acc))
    time.sleep(0.35)
    # Then rotate base
    base = target_pose.get(GRAB_BASE_SERVO_ID)
    if base is not None:
        ok_all = ok_all and sts_move_pos(GRAB_BASE_SERVO_ID, int(base), speed=int(speed), acc=int(acc))
    time.sleep(HOME_SETTLE_SEC)
    return ok_all


def _move_arm_to_pose(target_pose: dict[int, int], speed: int = HOME_MOVE_SPEED, acc: int = HOME_MOVE_ACC) -> bool:
    """Move all STS servos to a target pose dict {id: position}."""
    ok_all = True
    for sid in STS_SERVO_IDS:
        target = target_pose.get(sid)
        if target is None:
            ok_all = False
            continue
        ok = sts_move_pos(sid, int(target), speed=int(speed), acc=int(acc))
        ok_all = ok_all and ok
    time.sleep(HOME_SETTLE_SEC)
    return ok_all


def _state_machine_worker():
    """Runs grab and release sequences with timed waits."""
    global _state, _slot_grab_count, _grab_offset_warn_ts
    while True:
        time.sleep(0.08)
        with _state_lock:
            s = _state
        if s == STATE_SETUP:
            # Avoid overlapping the startup STS/LED routine with the setup FSM.
            if not _startup_check_done.is_set():
                continue
            _handle_setup_phase()
        elif s == STATE_GRAB:
            with _state_lock:
                _state = "grab_running"
            # Ensure UI + LED matrix reflect grab state (socket + matrix).
            try:
                ui.send_message("state", message={"state": STATE_GRAB})
                led_matrix_set_state_name(STATE_GRAB)
            except Exception:
                pass
            if not FIRST_SLOT_POSITION:
                print("[GRAB] First slot not configured yet.")
                core_matrix_set_error_code(ERR_GRAB_SLOT_NOT_CONFIGURED)
                _set_state(STATE_SETUP)
                continue

            slot_idx = _slot_grab_count % GRAB_SLOT_COUNT_MAX
            target_pose = dict(FIRST_SLOT_POSITION)
            base_raw = int(target_pose.get(GRAB_BASE_SERVO_ID, 2048))

            # Warn if default base offset likely causes clamping/duplicates.
            now_ts = time.time()
            if now_ts - _grab_offset_warn_ts > 20.0:
                positions = []
                clamped_low = 0
                clamped_high = 0
                for i in range(GRAB_SLOT_COUNT_MAX):
                    v = base_raw + int(GRAB_BASE_OFFSET_STEP) * int(i) * int(GRAB_BASE_OFFSET_DIRECTION)
                    v = max(0, min(4095, v))
                    if v == 0:
                        clamped_low += 1
                    if v == 4095:
                        clamped_high += 1
                    positions.append(v)
                unique_count = len(set(positions))
                if unique_count < GRAB_SLOT_COUNT_MAX or clamped_low or clamped_high:
                    print(
                        "[GRAB][WARN] base offset might be too aggressive: "
                        f"unique_positions={unique_count}/{GRAB_SLOT_COUNT_MAX}, "
                        f"clamped_low={clamped_low}, clamped_high={clamped_high}. "
                        "Consider lowering GRAB_BASE_OFFSET_STEP or adjusting direction."
                    )
                    _grab_offset_warn_ts = now_ts

            offset = int(GRAB_BASE_OFFSET_STEP) * int(slot_idx) * int(GRAB_BASE_OFFSET_DIRECTION)
            target_pose[GRAB_BASE_SERVO_ID] = max(0, min(4095, base_raw + offset))

            # Grab approach:
            # 1) Rotate base only toward slot
            # 2) Partial descent (arm halfway)
            # 3) Set narrow gripper opening
            # 4) Full descent to slot
            # 5) Close gripper max and hold tight
            _set_gripper_hold(True)

            # Step 1: rotate base to face slot
            base_target = target_pose.get(GRAB_BASE_SERVO_ID)
            if base_target is not None:
                sts_move_pos(GRAB_BASE_SERVO_ID, int(base_target), speed=GRAB_APPROACH_SPEED, acc=GRAB_APPROACH_ACC)
            time.sleep(1.0)

            # Step 2: partial descent — arm joints halfway between home and target
            for sid in STS_SERVO_IDS:
                if sid == GRAB_BASE_SERVO_ID:
                    continue
                home_val = int(HOME_POSITION.get(sid, 2048))
                target_val = int(target_pose.get(sid, home_val))
                mid = _clamp_pos(int(home_val + 0.5 * (target_val - home_val)))
                sts_move_pos(sid, mid, speed=GRAB_APPROACH_SPEED, acc=GRAB_APPROACH_ACC)
            time.sleep(0.8)

            # Step 3: set narrow gripper opening
            _move_servo(GRIPPER_GRAB_PREOPEN_ANGLE)
            time.sleep(0.3)

            # Step 4: full descent to slot position
            for sid in STS_SERVO_IDS:
                if sid == GRAB_BASE_SERVO_ID:
                    continue
                target = target_pose.get(sid)
                if target is not None:
                    sts_move_pos(sid, int(target), speed=GRAB_APPROACH_SPEED, acc=GRAB_APPROACH_ACC)
            time.sleep(GRAB_SETTLE_BEFORE_CLOSE_SEC)

            p_open = get_pressure()
            print(f"[GRAB][PRESSURE][open] A0={p_open.get('a0')} A1={p_open.get('a1')}")

            # Step 5: close gripper fully and hold tight (retry — critical)
            for _grip_try in range(3):
                _move_servo(GRIPPER_CLOSED_ANGLE)
                time.sleep(0.3)
            time.sleep(GRAB_CLOSE_WAIT_SEC)
            p_closed = get_pressure()
            print(f"[GRAB][PRESSURE][after_pick] A0={p_closed.get('a0')} A1={p_closed.get('a1')}")
            _move_arm_then_base(HOME_POSITION, speed=GRAB_RETURN_SPEED, acc=GRAB_RETURN_ACC, lift_pose=HOME_POSITION)

            # Delivery: from HOME, reach forward and present gadget.
            delivery_pose = _get_delivery_pose()
            _move_arm_to_pose(delivery_pose, speed=DELIVERY_SPEED, acc=DELIVERY_ACC)

            # Wait for a face to appear (user coming to take the pen).
            _delivery_face_seen.clear()
            with _state_lock:
                _state = "delivery_waiting"
            led_matrix_set_state_name(STATE_GRAB)  # keep grab eyes while waiting
            print("[DELIVERY] Waiting for face to hand over pen...")

            _delivery_face_seen.wait()  # blocks until face_detected() signals
            print("[DELIVERY] Face detected! Showing heart eyes...")

            # Heart eyes for 3 seconds, then release
            led_matrix_set_state_name(STATE_DELIVERY)
            time.sleep(DELIVERY_HEART_SEC)

            # Release pen
            _move_servo(GRIPPER_OPEN_ANGLE)
            time.sleep(0.8)
            _move_servo(GRIPPER_CLOSED_ANGLE)
            time.sleep(0.6)

            # Return to HOME
            _move_arm_to_pose(HOME_POSITION, speed=DELIVERY_SPEED, acc=DELIVERY_ACC)
            _slot_grab_count = (_slot_grab_count + 1) % GRAB_SLOT_COUNT_MAX
            print(f"[GRAB] completed slot #{_slot_grab_count if _slot_grab_count > 0 else GRAB_SLOT_COUNT_MAX}")

            # Wait at home before returning to idle
            print(f"[DELIVERY] Waiting {DELIVERY_POST_RELEASE_HOME_SEC}s before idle...")
            time.sleep(DELIVERY_POST_RELEASE_HOME_SEC)
            _set_state(STATE_DETECT)
        elif s == STATE_RELEASE:
            with _state_lock:
                _state = "release_running"
            # Ensure UI + LED matrix reflect release state (socket + matrix).
            try:
                ui.send_message("state", message={"state": STATE_RELEASE})
                led_matrix_set_state_name(STATE_RELEASE)
            except Exception:
                pass
            _move_servo(GRIPPER_OPEN_ANGLE)
            time.sleep(RELEASE_OPEN_WAIT_SEC)
            _set_state(STATE_DETECT)


# Home iniziale: verrà acquisita dalla posa corrente allo startup check.


def _refresh_idle_base_positions():
    """Aggiorna le posizioni di base per l'animazione idle, leggendo i servo attuali."""
    global _idle_base_pos, _idle_traj_state
    bases: dict[int, int] = {}
    traj: dict[int, dict[str, float]] = {}
    for sid in STS_SERVO_IDS:
        pos = sts_read_pos(sid)
        if pos is not None:
            bases[sid] = pos
            traj[sid] = {
                "current": float(pos),
                "last_sent": float(pos),
                "phase": float((sid - 1) * (math.pi / 4.0)),
                "last_cmd_ts": 0.0,
            }
    with _idle_lock:
        _idle_base_pos = bases
        _idle_traj_state = traj


def _slew_towards(current: float, target: float, max_step: float) -> float:
    """Move current toward target with bounded step for smooth trajectories."""
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return current + (max_step if delta > 0 else -max_step)


def _idle_animation_worker():
    """Animazione 'cerca qualcuno' quando il robot è in stato detect."""
    global _idle_pause_until, _idle_traj_state
    t0 = time.time()
    last_state = None
    detect_cycle_start_ts = 0.0
    rest_home_done = False
    while True:
        time.sleep(IDLE_ANIM_CHECK_INTERVAL_SEC)
        if not IDLE_ANIM_ENABLED:
            continue

        now = time.time()
        if now < _idle_pause_until:
            # Pausa temporanea (es. dopo un comando manuale dagli slider)
            continue

        with _state_lock:
            s = _state

        if s != STATE_DETECT:
            last_state = s
            # In grab/release we don't need aggressive idle polling; reduce CPU noise
            # to avoid starving other threads (e.g. websocket keepalive).
            time.sleep(0.2)
            continue

        # Siamo entrati (o rientrati) nello stato detect: aggiorna le posizioni di base
        if last_state != STATE_DETECT:
            _set_gripper_idle_closed_free()
            _refresh_idle_base_positions()
            detect_cycle_start_ts = now
            rest_home_done = False
        last_state = s

        # Idle duty cycle: 10s animation + 30s home rest (repeat).
        cycle_len = float(IDLE_ANIM_ACTIVE_WINDOW_SEC + IDLE_ANIM_REST_WINDOW_SEC)
        elapsed = now - detect_cycle_start_ts
        if cycle_len > 0.0 and elapsed >= cycle_len:
            detect_cycle_start_ts = now
            elapsed = 0.0
            rest_home_done = False

        if elapsed >= float(IDLE_ANIM_ACTIVE_WINDOW_SEC):
            if not rest_home_done:
                _move_arm_to_home_position()
                rest_home_done = True
            continue

        with _idle_lock:
            bases = dict(_idle_base_pos)
            traj = dict(_idle_traj_state)
        if not bases:
            continue

        # Oscillazione morbida con slew-rate limit e deadband per evitare scatti.
        t = now - t0
        for sid, base in bases.items():
            state = traj.get(sid)
            if state is None:
                continue
            amp = IDLE_ANIM_AMPLITUDES.get(sid, 150)
            phase = state.get("phase", 0.0)
            wave = math.sin(2.0 * math.pi * t / IDLE_ANIM_PERIOD_SEC + phase)
            desired = float(base) + float(amp) * wave
            desired = max(0.0, min(4095.0, desired))

            current = state.get("current", float(base))
            filtered = current + float(IDLE_ANIM_FILTER_ALPHA) * (desired - current)
            smoothed = _slew_towards(filtered, desired, float(IDLE_ANIM_MAX_STEP))
            smoothed = max(0.0, min(4095.0, smoothed))

            last_sent = state.get("last_sent", float(base))
            last_cmd_ts = state.get("last_cmd_ts", 0.0)
            can_send = (now - last_cmd_ts) >= float(IDLE_ANIM_MIN_CMD_INTERVAL_SEC)
            if can_send and abs(smoothed - last_sent) >= float(IDLE_ANIM_DEADBAND):
                target = int(round(smoothed))
                ok = sts_move_pos(sid, target, speed=IDLE_ANIM_SPEED, acc=IDLE_ANIM_ACC)
                if ok:
                    state["last_sent"] = float(target)
                    state["last_cmd_ts"] = now
                else:
                    # Se il comando fallisce, evitiamo di accumulare troppo errore.
                    state["last_sent"] = smoothed

            state["current"] = smoothed
            traj[sid] = state

        with _idle_lock:
            _idle_traj_state = traj


def face_detected():
    """In detect state: transition to grab. In delivery state: signal face seen."""
    with _state_lock:
        s = _state
    if s == STATE_DETECT:
        if time.time() < _detect_cooldown_until:
            return
        _set_state(STATE_GRAB)
    elif s == STATE_DELIVERY or s == "delivery_waiting":
        _delivery_face_seen.set()


#
# WebUI disabilitata: override_th (slider UI) non viene più gestita.
#


def get_pressure():
    """GET /api/pressure: return pressure sensor values from A0 and A1."""
    try:
        a0 = _bridge_call_and_unwrap("read_pressure_a0")
        a1 = _bridge_call_and_unwrap("read_pressure_a1")
        return {
            "a0": int(a0) if a0 is not None else None,
            "a1": int(a1) if a1 is not None else None,
        }
    except Exception:
        core_matrix_set_error_code(ERR_PRESSURE_READ_FAILED)
        return {"a0": None, "a1": None}


def _state_for_ui() -> str:
    """Map internal machine state to user-facing state value."""
    runtime_map = {
        "grab_running": STATE_GRAB,
        "delivery_waiting": STATE_DELIVERY,
        "release_running": STATE_RELEASE,
    }
    with _state_lock:
        s = _state
    if s in (STATE_SETUP, STATE_DETECT, STATE_GRAB, STATE_DELIVERY, STATE_RELEASE):
        return s
    return runtime_map.get(s, STATE_DETECT)


def get_state():
    """GET /api/state: return current state machine state (detect / grab / release)."""
    return {"state": _state_for_ui()}


def get_led_matrix_status():
    """GET /api/led_matrix_status: availability + current intensity."""
    return {
        "available": led_matrix_available(),
        "intensity": _led_matrix_intensity,
    }


def set_led_matrix_intensity(value=None):
    """GET /api/led_matrix_intensity?value=0..15."""
    global _led_matrix_intensity
    try:
        level = _get_request_arg("value", value)
        if level is None:
            return {"ok": False, "error": "value required"}
        level = int(level)
        if level < 0 or level > 15:
            return {"ok": False, "error": "value must be 0..15"}
        ok = led_matrix_set_intensity(level)
        if ok:
            _led_matrix_intensity = level
        return {"ok": ok, "intensity": _led_matrix_intensity}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def clear_led_matrix():
    """GET /api/led_matrix_clear."""
    try:
        return {"ok": led_matrix_clear()}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _enter_setup_mode():
    global _setup_phase, _setup_prev_pose, _setup_stable_since, _setup_phase_moved, _setup_torque_free_active
    global FIRST_SLOT_POSITION, SECOND_SLOT_POSITION, _slot_grab_count
    global _setup_last_blink_ts, _setup_blink_on, _setup_anchor_pose
    _setup_phase = "home"
    _setup_prev_pose = None
    _setup_anchor_pose = None
    _setup_stable_since = time.time()
    _setup_phase_moved = False
    _setup_last_blink_ts = 0.0
    _setup_blink_on = True
    _set_sts_torque_enabled(False)
    _setup_torque_free_active = True
    FIRST_SLOT_POSITION = {}
    SECOND_SLOT_POSITION = {}
    _slot_grab_count = 0
    _set_state(STATE_SETUP)
    core_matrix_set_setup_step(1)  # S1 = move to HOME


def set_state_checkpoint(state=None):
    """GET /api/set_state?state=setup|detect|grab|release: force state machine checkpoint."""
    try:
        new_state = _get_request_arg("state", state)
        if new_state is None:
            return {"ok": False, "error": "state required"}
        new_state = str(new_state).strip().lower()
        if new_state not in (STATE_SETUP, STATE_DETECT, STATE_GRAB, STATE_RELEASE):
            return {"ok": False, "error": "state must be setup|detect|grab|release"}
        if new_state == STATE_SETUP:
            _enter_setup_mode()
            return {"ok": True, "state": STATE_SETUP}
        _set_state(new_state)
        return {"ok": True, "state": new_state}
    except Exception as e:
        return {"ok": False, "error": str(e)}


#
# WebUI disabilitata: nessuna expose_api registrata.
#


def get_servo_positions():
    """GET /api/servo_positions: posizioni attuali dei 4 servo STS3215 (per inizializzare gli slider)."""
    out = {}
    for sid in STS_SERVO_IDS:
        pos = sts_read_pos(sid)
        out[str(sid)] = int(pos) if pos is not None else None
    return out


def set_home_current():
    """GET /api/set_home_current: salva la posizione attuale dei servo come home (RAM)."""
    global HOME_POSITION
    valid = _capture_current_pose()
    if not valid:
        return {"ok": False, "error": "current servo pose unavailable or invalid"}

    HOME_POSITION = dict(valid)
    payload = _pose_payload(HOME_POSITION)
    return {"ok": True, "home": payload}


def set_first_slot_current():
    """GET /api/set_first_slot_current: salva in RAM la posa corrente del primo slot."""
    global FIRST_SLOT_POSITION
    valid = _capture_current_pose()
    if not valid:
        return {"ok": False, "error": "current servo pose unavailable or invalid"}

    FIRST_SLOT_POSITION = dict(valid)
    payload = _pose_payload(FIRST_SLOT_POSITION)
    return {"ok": True, "first_slot": payload}


def go_to_home():
    """GET /api/go_home: move robot arm to persisted HOME_POSITION."""
    _pause_idle_temporarily()
    ok = _move_arm_to_pose(HOME_POSITION)
    return {"ok": ok, "home": _pose_payload(HOME_POSITION)}


def go_to_first_slot():
    """GET /api/go_first_slot: move robot arm to FIRST_SLOT_POSITION stored in RAM."""
    if not FIRST_SLOT_POSITION:
        return {"ok": False, "error": "first slot not saved yet"}
    _pause_idle_temporarily()
    ok = _move_arm_to_pose(FIRST_SLOT_POSITION)
    return {"ok": ok, "first_slot": _pose_payload(FIRST_SLOT_POSITION)}


def grab_offset_config(step=None, direction=None):
    """GET /api/grab_offset_config?step=55&direction=1 (direction: 1 or -1)."""
    global GRAB_BASE_OFFSET_STEP, GRAB_BASE_OFFSET_DIRECTION
    try:
        q_step = _get_request_arg("step", step)
        q_dir = _get_request_arg("direction", direction)
        if q_step is not None:
            q_step = int(q_step)
            if q_step < 0 or q_step > 500:
                return {"ok": False, "error": "step must be 0..500"}
            GRAB_BASE_OFFSET_STEP = q_step
        if q_dir is not None:
            q_dir = int(q_dir)
            if q_dir not in (-1, 1):
                return {"ok": False, "error": "direction must be -1 or 1"}
            GRAB_BASE_OFFSET_DIRECTION = q_dir

        return {
            "ok": True,
            "step": int(GRAB_BASE_OFFSET_STEP),
            "direction": int(GRAB_BASE_OFFSET_DIRECTION),
            "slot_count": int(_slot_grab_count),
            "slot_max": int(GRAB_SLOT_COUNT_MAX),
        }
    except Exception as e:
        return {"ok": False, "error": str(e)}


def servo_move(id=None, position=None):
    """GET /api/servo_move?id=1&position=2048: muove il servo dato alla posizione (0–4095)."""
    try:
        sid = _get_request_arg("id", id)
        pos = _get_request_arg("position", position)
        if sid is None or pos is None:
            return {"ok": False, "error": "id and position required"}
        sid = int(sid)
        pos = int(pos)
        if sid not in STS_SERVO_IDS:
            return {"ok": False, "error": f"id must be in {list(STS_SERVO_IDS)}"}
        if not 0 <= pos <= 4095:
            return {"ok": False, "error": "position must be 0–4095"}

        # Piccola pausa dell'animazione idle, così il comando manuale non viene subito sovrascritto
        _pause_idle_temporarily()

        # Come nello script funzionante: speed e acc non zero (es. 1500, 50)
        ok = sts_move_pos(sid, pos, speed=1500, acc=50)
        return {"ok": ok}
    except Exception as e:
        return {"ok": False, "error": str(e)}


#
# WebUI disabilitata: nessuna expose_api registrata.
#


def _bridge_call_and_unwrap(method: str, *args):
    """Helper: call a Bridge method and unwrap async result objects if needed.
    Uses _bridge_lock to prevent concurrent calls from multiple threads timing out."""
    def _is_transient_bridge_error(msg: str) -> bool:
        msg = msg.lower()
        # Heuristics for common startup/transient transport failures.
        markers = (
            "timed out",
            "timeout",
            "temporar",
            "connection",
            "disconnected",
            "closed",
            "broken pipe",
            "reset",
            "not ready",
            "not connected",
            "unavailable",
            "busy",
            "transport",
        )
        return any(m in msg for m in markers)

    for attempt in range(BRIDGE_CALL_RETRIES):
        with _bridge_lock:
            try:
                res = Bridge.call(method, *args)
                if hasattr(res, "result"):
                    try:
                        return res.result(timeout=BRIDGE_RESULT_TIMEOUT_SEC)
                    except Exception as e:
                        msg = str(e)
                        if attempt < (BRIDGE_CALL_RETRIES - 1) and _is_transient_bridge_error(msg):
                            pass  # will sleep outside the lock below
                        else:
                            return None
                else:
                    return res
            except Exception as e:
                msg = str(e)
                if attempt < (BRIDGE_CALL_RETRIES - 1) and _is_transient_bridge_error(msg):
                    pass  # will sleep outside the lock below
                else:
                    return None
        # Sleep OUTSIDE the lock so other threads are not blocked 30s during retries.
        time.sleep(BRIDGE_CALL_RETRY_DELAY_SEC)
    return None


def sts_ping(servo_id: int) -> bool:
    """Ping STS3215 with given ID. Returns True if the servo responds."""
    try:
        res = _bridge_call_and_unwrap("sts_ping", int(servo_id))
        if res is None:
            return False
        # SCServo Ping returns >=0 on success, <0 on failure.
        return int(res) >= 0
    except Exception as e:
        print(f"[STS3215] ping error for id={servo_id}: {e}")
        return False


def core_matrix_set_setup_step(step: int) -> bool:
    """Show setup step S1/S2/S3 on UNO Q onboard matrix."""
    try:
        res = _bridge_call_and_unwrap("core_matrix_set_setup_step", int(step))
        return res is not None and int(res) > 0
    except Exception as e:
        print(f"[MATRIX] core_matrix_set_setup_step({step}) error: {e}")
        return False


def core_matrix_clear() -> bool:
    """Clear the UNO Q onboard matrix (blank frame)."""
    try:
        res = _bridge_call_and_unwrap("core_matrix_clear")
        return res is not None and int(res) > 0
    except Exception:
        return False


def core_matrix_set_error_code(code: int) -> bool:
    """Show error 'ER' + numeric code on UNO Q onboard matrix."""
    try:
        res = _bridge_call_and_unwrap("core_matrix_set_error_code", int(code))
        return res is not None and int(res) > 0
    except Exception as e:
        print(f"[MATRIX] core_matrix_set_error_code({code}) error: {e}")
        return False


def sts_read_pos(servo_id: int) -> int | None:
    """Read raw position from STS3215 (or None on error)."""
    try:
        res = _bridge_call_and_unwrap("sts_read_pos", int(servo_id))
        if res is None:
            return None
        v = int(res)
        # SCServo returns negative on error.
        if v < 0:
            return None
        return v
    except Exception as e:
        print(f"[STS3215] read_pos error for id={servo_id}: {e}")
        return None


def sts_move_pos(servo_id: int, position: int, speed: int = 0, acc: int = 0) -> bool:
    """Move STS3215 to a given raw position with optional speed/acc."""
    try:
        res = _bridge_call_and_unwrap(
            "sts_move_pos", int(servo_id), int(position), int(speed), int(acc)
        )
        if res is None:
            return False
        return int(res) >= 0
    except Exception as e:
        print(f"[STS3215] move_pos error for id={servo_id}: {e}")
        core_matrix_set_error_code(ERR_STS_MOVE_FAILED)
        return False


def sts_set_torque(servo_id: int, enabled: bool) -> bool:
    """Enable/disable torque on one STS3215 servo."""
    try:
        en = 1 if enabled else 0
        res = _bridge_call_and_unwrap("sts_torque_enable", int(servo_id), int(en))
        if res is None:
            return False
        return int(res) >= 0
    except Exception as e:
        print(f"[STS3215] torque error for id={servo_id}: {e}")
        return False


def led_matrix_available() -> bool:
    """True if matrix module is available on MCU side."""
    try:
        res = _bridge_call_and_unwrap("led_matrix_available")
        return res is not None and int(res) > 0
    except Exception:
        return False


def led_matrix_set_intensity(level: int) -> bool:
    try:
        res = _bridge_call_and_unwrap("led_matrix_set_intensity", int(level))
        return res is not None and int(res) > 0
    except Exception:
        return False


def led_matrix_clear() -> bool:
    try:
        res = _bridge_call_and_unwrap("led_matrix_clear")
        return res is not None and int(res) > 0
    except Exception:
        return False


def led_matrix_set_state_name(state_name: str) -> bool:
    code_map = {
        STATE_SETUP: 3,
        STATE_DETECT: 0,
        STATE_GRAB: 1,
        STATE_RELEASE: 2,
        STATE_DELIVERY: 4,
    }
    code = code_map.get(str(state_name).lower())
    if code is None:
        return False
    try:
        res = _bridge_call_and_unwrap("led_matrix_set_state", int(code))
        return res is not None and int(res) > 0
    except Exception:
        return False

# Support both "face" and "Face" (model may use either)
detection_stream.on_detect("face", face_detected)
try:
    detection_stream.on_detect("Face", face_detected)
except Exception:
    pass


def _normalize_detection_item(key: str, value) -> list[tuple[str, float]]:
    """Return list of (content, confidence). Handles value as number, string, list of dicts or mixed."""
    if value is None:
        return []
    if isinstance(value, (int, float)):
        return [(key, float(value))]
    if isinstance(value, str):
        try:
            return [(key, float(value))]
        except (TypeError, ValueError):
            return []
    if isinstance(value, list):
        out = []
        for item in value:
            if isinstance(item, dict):
                conf = item.get("confidence")
                if conf is not None:
                    out.append((key, float(conf)))
            elif isinstance(item, (int, float)):
                out.append((key, float(item)))
            elif isinstance(item, str):
                try:
                    out.append((key, float(item)))
                except (TypeError, ValueError):
                    pass
        return out
    return []


def send_detections_to_ui(detections: dict):
    for key, values in detections.items():
        items = _normalize_detection_item(key, values)
        for content, confidence in items:
            entry = {
                "content": content,
                "confidence": confidence,
                "timestamp": datetime.now(UTC).isoformat(),
            }
            # Web UI disabilitata: non inviamo detections all'interfaccia web.
        # Move servo when we see a face (in case on_detect used different label)
        if key.lower() == "face" and items and items[0][1] >= 0.85:
            face_detected()


detection_stream.on_detect_all(send_detections_to_ui)

# LED matrix init moved to _startup_sts_check (Bridge not ready before App.run).

# Start workers after all symbols/functions are defined.
threading.Thread(target=_state_machine_worker, daemon=True).start()
threading.Thread(target=_idle_animation_worker, daemon=True).start()

# Check STS3215 all'avvio (dopo qualche secondo, così il Bridge è pronto)
threading.Thread(target=_startup_sts_check, daemon=True).start()

App.run()
