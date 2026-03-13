# SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL (http://www.arduino.cc)
#
# SPDX-License-Identifier: MPL-2.0

from arduino.app_utils import App, Bridge
from arduino.app_bricks.web_ui import WebUI
from arduino.app_bricks.video_objectdetection import VideoObjectDetection
from datetime import datetime, UTC
import time
import threading
import math

# ID dei servo STS3215 (solo 4 presenti sul bus)
STS_SERVO_IDS = (1, 2, 3, 4)
STS_CHECK_IDS = STS_SERVO_IDS
STS_STARTUP_DELAY_SEC = 3.0  # Attesa prima del check (Bridge deve essere pronto)



def _print_sts_servo_info():
    """Stampa a console ping e posizione per ogni ID STS3215 (per check)."""
    print("[STS3215] Check servo (ping + posizione):\n")
    for sid in STS_CHECK_IDS:
        ok = sts_ping(sid)
        pos = sts_read_pos(sid)
        pos_str = str(pos) if pos is not None else "—"
        status = "OK" if ok else "NO"
        print(f"  ID={sid}  ping={status}  pos={pos_str}")
    print()


def _startup_sts_check():
    """Esegue il check STS dopo un breve delay (Bridge deve essere connesso)."""
    time.sleep(STS_STARTUP_DELAY_SEC)
    try:
        print("[SETUP] Move robot to HOME, then hold still 5s.")
        print("[SETUP] Then move robot to FIRST_SLOT, then hold still 5s.")
        _set_gripper_idle_closed_free()
        _print_sts_servo_info()
        print(f"[MATRIX] available={led_matrix_available()}")
    except Exception as e:
        print(f"[STS3215] startup check failed: {e}")


# State machine: setup -> detect -> (face) -> grab -> detect
STATE_SETUP = "setup"
STATE_DETECT = "detect"
STATE_GRAB = "grab"
STATE_RELEASE = "release"

# Gripper servo angles (adjust if your servo is reversed)
GRIPPER_OPEN_ANGLE = 0
GRIPPER_CLOSED_ANGLE = 90
GRIPPER_GRAB_PREOPEN_ANGLE = 25  # "open a little" before slot approach

GRAB_OPEN_WAIT_SEC = 1.0
GRAB_CLOSE_WAIT_SEC = 1.0
GRAB_WAIT_BEFORE_HOME_SEC = 15.0
RELEASE_OPEN_WAIT_SEC = 3.0

# Posizione "home" del braccio STS3215 (RAM-only session).
HOME_DEFAULT_POSITION = {
    1: 2048,
    2: 2048,
    3: 2048,
    4: 2048,
}
HOME_POSITION = dict(HOME_DEFAULT_POSITION)
HOME_MOVE_SPEED = 1200
HOME_MOVE_ACC = 40
HOME_SETTLE_SEC = 1.2

# Primo slot (RAM-only session)
FIRST_SLOT_POSITION: dict[int, int] = {}

# Setup automatico: fermo per N secondi -> salva posizione.
SETUP_STABLE_SECONDS = 5.0
SETUP_MOVEMENT_THRESHOLD = 25
SETUP_NOD_DELTA_SERVO4 = 120
SETUP_NOD_SPEED = 500
SETUP_NOD_ACC = 20
SETUP_NOD_DELAY_SEC = 0.30

# Grab sequenziale su 9 slot con offset base servo1 configurabile.
GRAB_SLOT_COUNT_MAX = 9
GRAB_BASE_SERVO_ID = 1
GRAB_BASE_OFFSET_STEP = 300
GRAB_BASE_OFFSET_DIRECTION = -1  # +1 o -1

ui = WebUI()
detection_stream = VideoObjectDetection(confidence=0.85, debounce_sec=0.0)

_state = STATE_SETUP
_state_lock = threading.Lock()
_setup_phase = "home"  # home -> first_slot -> done
_setup_prev_pose: dict[int, int] | None = None
_setup_stable_since = 0.0
_setup_phase_moved = False  # diventa True quando rileva movimento manuale nella fase corrente
_setup_torque_free_active = False
_slot_grab_count = 0

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

# Push realtime UI telemetry (socket events)
TELEMETRY_READER_INTERVAL_SEC = 0.10
TELEMETRY_PUSH_INTERVAL_SEC = 0.10
TELEMETRY_STATE_HEARTBEAT_SEC = 2.0
TELEMETRY_LOG_INTERVAL_SEC = 2.0
LED_MATRIX_DEFAULT_INTENSITY = 3

_idle_base_pos: dict[int, int] = {}
_idle_traj_state: dict[int, dict[str, float]] = {}
_idle_lock = threading.Lock()
_idle_pause_until = 0.0  # timestamp: finché > now, animazione sospesa (es. dopo comando manuale)
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
    current = {}
    for sid in STS_SERVO_IDS:
        pos = sts_read_pos(sid)
        if pos is None:
            return None
        current[sid] = int(pos)
    valid = _validated_home_position(current)
    return valid if valid else None


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
    """Auto setup: hold still 5s to capture HOME, then FIRST_SLOT."""
    global _setup_prev_pose, _setup_stable_since, _setup_phase, _setup_phase_moved, _setup_torque_free_active, HOME_POSITION, FIRST_SLOT_POSITION, _slot_grab_count
    now = time.time()
    if not _setup_torque_free_active:
        _set_sts_torque_enabled(False)
        _setup_torque_free_active = True
    pose = _read_current_pose()
    if pose is None:
        return

    if _setup_prev_pose is None:
        _setup_prev_pose = dict(pose)
        _setup_stable_since = now
        print(f"[SETUP] Phase={_setup_phase}: move arm by hand, then keep still {SETUP_STABLE_SECONDS:.0f}s.")
        return

    moved = _pose_has_movement(pose, _setup_prev_pose, SETUP_MOVEMENT_THRESHOLD)
    _setup_prev_pose = dict(pose)
    if moved:
        if not _setup_phase_moved:
            print(f"[SETUP] Phase={_setup_phase}: movement detected, waiting stable hold...")
        _setup_phase_moved = True
        _setup_stable_since = now
        return

    # Evita catture premature: la fase si arma solo dopo almeno un movimento manuale.
    if not _setup_phase_moved:
        return

    if (now - _setup_stable_since) < SETUP_STABLE_SECONDS:
        return

    if _setup_phase == "home":
        HOME_POSITION = dict(pose)
        print(f"[SETUP] HOME captured: {HOME_POSITION}")
        _set_sts_torque_enabled(True)
        _setup_torque_free_active = False
        _setup_nod_servo4()
        _set_sts_torque_enabled(False)
        _setup_torque_free_active = True
        _setup_phase = "first_slot"
        _setup_phase_moved = False
        _setup_prev_pose = None
        _setup_stable_since = now
        return

    if _setup_phase == "first_slot":
        FIRST_SLOT_POSITION = dict(pose)
        _slot_grab_count = 0
        print(f"[SETUP] FIRST_SLOT captured: {FIRST_SLOT_POSITION}")
        _setup_phase = "done"
        _setup_phase_moved = False
        _set_sts_torque_enabled(True)
        _setup_torque_free_active = False
        print("[SETUP] Returning to HOME before idle.")
        _move_arm_to_home_position()
        _set_state(STATE_DETECT)


def _set_state(new_state: str):
    global _state, _setup_torque_free_active
    old_state = _state
    if old_state == STATE_SETUP and new_state != STATE_SETUP and _setup_torque_free_active:
        _set_sts_torque_enabled(True)
        _setup_torque_free_active = False
    with _state_lock:
        _state = new_state
    try:
        ui.send_message("state", message={"state": new_state})
    except Exception:
        pass
    try:
        led_matrix_set_state_name(new_state)
    except Exception:
        pass
    print(f"[StateMachine] -> {new_state}")


def _move_servo(angle: int):
    try:
        Bridge.call("move_servo", angle)
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
    # Idle policy: close gripper, then release torque so it is manually movable.
    _set_gripper_hold(True)
    _move_servo(GRIPPER_CLOSED_ANGLE)
    time.sleep(0.2)
    _set_gripper_hold(False)


def _move_arm_to_home_position():
    """Muove tutti i servo STS verso la home prima del ritorno a idle."""
    for sid in STS_SERVO_IDS:
        target = HOME_POSITION.get(sid)
        if target is None:
            continue
        sts_move_pos(sid, int(target), speed=HOME_MOVE_SPEED, acc=HOME_MOVE_ACC)
    time.sleep(HOME_SETTLE_SEC)


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
    global _state, _slot_grab_count
    while True:
        time.sleep(0.08)
        with _state_lock:
            s = _state
        if s == STATE_SETUP:
            _handle_setup_phase()
        elif s == STATE_GRAB:
            with _state_lock:
                _state = "grab_running"
            if not FIRST_SLOT_POSITION:
                print("[GRAB] First slot not configured yet.")
                _set_state(STATE_SETUP)
                continue

            slot_idx = _slot_grab_count % GRAB_SLOT_COUNT_MAX
            target_pose = dict(FIRST_SLOT_POSITION)
            base_raw = int(target_pose.get(GRAB_BASE_SERVO_ID, 2048))
            offset = int(GRAB_BASE_OFFSET_STEP) * int(slot_idx) * int(GRAB_BASE_OFFSET_DIRECTION)
            target_pose[GRAB_BASE_SERVO_ID] = max(0, min(4095, base_raw + offset))

            # Grab sequence: small open -> go to slot (+offset) -> close max -> wait -> home (closed)
            _set_gripper_hold(True)
            _move_servo(GRIPPER_GRAB_PREOPEN_ANGLE)
            time.sleep(GRAB_OPEN_WAIT_SEC)
            _move_arm_to_pose(target_pose)
            p_open = get_pressure()
            print(f"[GRAB][PRESSURE][open] A0={p_open.get('a0')} A1={p_open.get('a1')}")
            _move_servo(GRIPPER_CLOSED_ANGLE)
            time.sleep(GRAB_CLOSE_WAIT_SEC)
            p_closed = get_pressure()
            print(f"[GRAB][PRESSURE][after_pick] A0={p_closed.get('a0')} A1={p_closed.get('a1')}")
            _move_arm_to_home_position()
            # Keep object clamped until we are back at HOME, then wait user pickup time.
            time.sleep(GRAB_WAIT_BEFORE_HOME_SEC)
            _move_servo(GRIPPER_OPEN_ANGLE)
            time.sleep(0.8)
            _move_servo(GRIPPER_CLOSED_ANGLE)
            time.sleep(0.6)
            _slot_grab_count = (_slot_grab_count + 1) % GRAB_SLOT_COUNT_MAX
            print(f"[GRAB] completed slot #{_slot_grab_count if _slot_grab_count > 0 else GRAB_SLOT_COUNT_MAX}")
            _set_state(STATE_DETECT)
        elif s == STATE_RELEASE:
            with _state_lock:
                _state = "release_running"
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
            continue

        # Siamo entrati (o rientrati) nello stato detect: aggiorna le posizioni di base
        if last_state != STATE_DETECT:
            _set_gripper_idle_closed_free()
            _refresh_idle_base_positions()
        last_state = s

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
    """In detect state: transition to grab when a face is detected."""
    with _state_lock:
        if _state != STATE_DETECT:
            return
    _set_state(STATE_GRAB)


ui.on_message("override_th", lambda sid, threshold: detection_stream.override_threshold(threshold))


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
        return {"a0": None, "a1": None}


def _state_for_ui() -> str:
    """Map internal machine state to user-facing state value."""
    runtime_map = {
        "grab_running": STATE_GRAB,
        "release_running": STATE_RELEASE,
    }
    with _state_lock:
        s = _state
    if s in (STATE_SETUP, STATE_DETECT, STATE_GRAB, STATE_RELEASE):
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
    global _setup_phase, _setup_prev_pose, _setup_stable_since, _setup_phase_moved, _setup_torque_free_active, FIRST_SLOT_POSITION, _slot_grab_count
    _setup_phase = "home"
    _setup_prev_pose = None
    _setup_stable_since = time.time()
    _setup_phase_moved = False
    _set_sts_torque_enabled(False)
    _setup_torque_free_active = True
    FIRST_SLOT_POSITION = {}
    _slot_grab_count = 0
    _set_state(STATE_SETUP)


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


ui.expose_api("GET", "/api/pressure", get_pressure)
ui.expose_api("GET", "/api/state", get_state)
ui.expose_api("GET", "/api/led_matrix_status", get_led_matrix_status)
ui.expose_api("GET", "/api/led_matrix_intensity", set_led_matrix_intensity)
ui.expose_api("GET", "/api/led_matrix_clear", clear_led_matrix)
ui.expose_api("GET", "/api/set_state", set_state_checkpoint)


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
    try:
        ui.send_message("home_updated", message={"home": payload})
    except Exception:
        pass
    return {"ok": True, "home": payload}


def set_first_slot_current():
    """GET /api/set_first_slot_current: salva in RAM la posa corrente del primo slot."""
    global FIRST_SLOT_POSITION
    valid = _capture_current_pose()
    if not valid:
        return {"ok": False, "error": "current servo pose unavailable or invalid"}

    FIRST_SLOT_POSITION = dict(valid)
    payload = _pose_payload(FIRST_SLOT_POSITION)
    try:
        ui.send_message("first_slot_updated", message={"first_slot": payload})
    except Exception:
        pass
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


ui.expose_api("GET", "/api/servo_positions", get_servo_positions)
ui.expose_api("GET", "/api/servo_move", servo_move)
ui.expose_api("GET", "/api/set_home_current", set_home_current)
ui.expose_api("GET", "/api/set_first_slot_current", set_first_slot_current)
ui.expose_api("GET", "/api/go_home", go_to_home)
ui.expose_api("GET", "/api/go_first_slot", go_to_first_slot)
ui.expose_api("GET", "/api/grab_offset_config", grab_offset_config)


def _bridge_call_and_unwrap(method: str, *args):
    """Helper: call a Bridge method and unwrap async result objects if needed."""
    res = Bridge.call(method, *args)
    if hasattr(res, "result"):
        try:
            return res.result()
        except Exception:
            return None
    return res


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


def sts_read_pos(servo_id: int) -> int | None:
    """Read raw position from STS3215 (or None on error)."""
    try:
        res = _bridge_call_and_unwrap("sts_read_pos", int(servo_id))
        if res is None:
            return None
        return int(res)
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
            ui.send_message("detection", message=entry)
        # Move servo when we see a face (in case on_detect used different label)
        if key.lower() == "face" and items and items[0][1] >= 0.85:
            face_detected()


detection_stream.on_detect_all(send_detections_to_ui)

# Initialize LED matrix (if present) with default state.
try:
    led_matrix_set_intensity(_led_matrix_intensity)
    led_matrix_set_state_name(STATE_SETUP)
except Exception:
    pass

# Start workers after all symbols/functions are defined.
threading.Thread(target=_state_machine_worker, daemon=True).start()
threading.Thread(target=_idle_animation_worker, daemon=True).start()

# Check STS3215 all'avvio (dopo qualche secondo, così il Bridge è pronto)
threading.Thread(target=_startup_sts_check, daemon=True).start()

App.run()
