# SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL (http://www.arduino.cc)
#
# SPDX-License-Identifier: MPL-2.0

from arduino.app_utils import App, Bridge
from arduino.app_bricks.web_ui import WebUI
from arduino.app_bricks.video_objectdetection import VideoObjectDetection
from datetime import datetime, UTC
import time
import threading

# ID dei servo STS3215 (solo 4 presenti sul bus)
STS_SERVO_IDS = (1, 2, 3, 4)
STS_CHECK_IDS = STS_SERVO_IDS
STS_STARTUP_DELAY_SEC = 3.0  # Attesa prima del check (Bridge deve essere pronto)


def clear_console():
    """Pulisce il terminale (ANSI, compatibile con UNO Q / Linux)."""
    print("\033[2J\033[H", end="")
    print("=" * 60)
    print("RUN", datetime.now(UTC).isoformat())
    print("=" * 60)


def _print_sts_servo_info():
    """Stampa a console ping e posizione per ogni ID STS3215 (per check)."""
    clear_console()
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
        _print_sts_servo_info()
    except Exception as e:
        print(f"[STS3215] startup check failed: {e}")


# State machine: detect -> (face) -> grab -> release -> detect
STATE_DETECT = "detect"
STATE_GRAB = "grab"
STATE_RELEASE = "release"

# Gripper servo angles (adjust if your servo is reversed)
GRIPPER_OPEN_ANGLE = 0
GRIPPER_CLOSED_ANGLE = 90

GRAB_OPEN_WAIT_SEC = 1.0
GRAB_CLOSE_WAIT_SEC = 1.0
RELEASE_OPEN_WAIT_SEC = 3.0

ui = WebUI()
detection_stream = VideoObjectDetection(confidence=0.5, debounce_sec=0.0)

_state = STATE_DETECT
_state_lock = threading.Lock()


def _set_state(new_state: str):
    global _state
    with _state_lock:
        _state = new_state
    try:
        ui.send_message("state", message={"state": new_state})
    except Exception:
        pass
    print(f"[StateMachine] -> {new_state}")


def _move_servo(angle: int):
    try:
        Bridge.call("move_servo", angle)
    except Exception as e:
        print(f"[StateMachine] move_servo({angle}) error: {e}")


def _state_machine_worker():
    """Runs grab and release sequences with timed waits."""
    global _state
    while True:
        time.sleep(0.2)
        with _state_lock:
            s = _state
        if s == STATE_GRAB:
            with _state_lock:
                _state = "grab_running"
            # Grab: open gripper, wait 1s, close, wait 1s, then release
            _move_servo(GRIPPER_OPEN_ANGLE)
            time.sleep(GRAB_OPEN_WAIT_SEC)
            _move_servo(GRIPPER_CLOSED_ANGLE)
            time.sleep(GRAB_CLOSE_WAIT_SEC)
            _set_state(STATE_RELEASE)
        elif s == STATE_RELEASE:
            with _state_lock:
                _state = "release_running"
            # Release: open gripper, wait 3s, back to detect
            _move_servo(GRIPPER_OPEN_ANGLE)
            time.sleep(RELEASE_OPEN_WAIT_SEC)
            _set_state(STATE_DETECT)


def face_detected():
    """In detect state: transition to grab when a face is detected."""
    with _state_lock:
        if _state != STATE_DETECT:
            return
    _set_state(STATE_GRAB)


# Start state machine worker thread
_worker = threading.Thread(target=_state_machine_worker, daemon=True)
_worker.start()

ui.on_message("override_th", lambda sid, threshold: detection_stream.override_threshold(threshold))


def get_pressure():
    """GET /api/pressure: return pressure sensor values from A0 and A1."""
    try:
        a0 = Bridge.call("read_pressure_a0")
        a1 = Bridge.call("read_pressure_a1")
        if hasattr(a0, "result"):
            a0 = a0.result()
        if hasattr(a1, "result"):
            a1 = a1.result()
        return {"a0": a0, "a1": a1}
    except Exception as e:
        print(f"[API] pressure error: {e}")
        return {"a0": None, "a1": None, "error": str(e)}


def get_state():
    """GET /api/state: return current state machine state (detect / grab / release)."""
    with _state_lock:
        s = _state
    if s in (STATE_DETECT, STATE_GRAB, STATE_RELEASE):
        return {"state": s}
    if s == "grab_running":
        return {"state": STATE_GRAB}
    if s == "release_running":
        return {"state": STATE_RELEASE}
    return {"state": STATE_DETECT}


ui.expose_api("GET", "/api/pressure", get_pressure)
ui.expose_api("GET", "/api/state", get_state)


def get_servo_positions():
    """GET /api/servo_positions: posizioni attuali dei 4 servo STS3215 (per inizializzare gli slider)."""
    out = {}
    for sid in STS_SERVO_IDS:
        pos = sts_read_pos(sid)
        out[str(sid)] = pos if pos is not None else None
    return out


def servo_move():
    """GET /api/servo_move?id=1&position=2048: muove il servo dato alla posizione (0–4095)."""
    try:
        from flask import request
        sid = request.args.get("id")
        pos = request.args.get("position")
        if sid is None or pos is None:
            return {"ok": False, "error": "id and position required"}
        sid = int(sid)
        pos = int(pos)
        if sid not in STS_SERVO_IDS:
            return {"ok": False, "error": f"id must be in {list(STS_SERVO_IDS)}"}
        if not 0 <= pos <= 4095:
            return {"ok": False, "error": "position must be 0–4095"}
        ok = sts_move_pos(sid, pos)
        return {"ok": ok}
    except Exception as e:
        return {"ok": False, "error": str(e)}


ui.expose_api("GET", "/api/servo_positions", get_servo_positions)
ui.expose_api("GET", "/api/servo_move", servo_move)


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
        if key.lower() == "face" and items and items[0][1] >= 0.5:
            face_detected()


detection_stream.on_detect_all(send_detections_to_ui)

# Check STS3215 all'avvio (dopo qualche secondo, così il Bridge è pronto)
threading.Thread(target=_startup_sts_check, daemon=True).start()

App.run()
