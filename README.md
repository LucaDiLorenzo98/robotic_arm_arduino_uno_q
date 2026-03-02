# Face Detection Servo – Arduino UNO Q

Move a servo on **digital pin 9** when a face is detected. Uses Video Object Detection, Web UI, and Bridge (MCU/MPU).

## Build requirement: MsgPack library

If the sketch fails with:

```text
MsgPack.h: No such file or directory
```

then the **MsgPack** library is missing. The Bridge (Arduino_RouterBridge) uses Arduino_RPClite, which depends on **MsgPack** (≥0.4.2). The board package sometimes does not install it automatically.

### Fix

1. In **Arduino Lab** (or Arduino IDE): open **Tools → Manage Libraries** (or **Sketch → Include Library → Manage Libraries**).
2. Search for **MsgPack**.
3. Install **MsgPack** by **hideakitai** (version **0.4.2** or newer).
4. Build/upload the sketch again.

If Arduino Lab does not offer Library Manager, install the library manually:

- Download: [MsgPack by hideakitai](https://github.com/hideakitai/MsgPack) (e.g. as ZIP from GitHub).
- Extract and copy the `MsgPack` folder into your Arduino **libraries** folder (e.g. `Documents/Arduino/libraries` on Windows, or the libraries folder used by Arduino Lab).
- Restart Arduino Lab and build again.

## Usage

1. Open the app in Arduino Lab and flash the board (sketch + Python app).
2. Connect a servo to **digital pin 9** (signal), 5 V, and GND.
3. Open the Web UI: left = live camera with face detection; right = feedback and recent detections.
4. When a face is detected, the servo moves to 90° and the UI shows “Face detected!”.
