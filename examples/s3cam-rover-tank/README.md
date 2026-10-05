# Rover / Tank: CYD controller + ESP32-S3 CAM

Two projects that work together (ESP-NOW only, no WiFi/AP):

| Project | Env | Source |
|---|---|---|
| ESP32-S3 CAM tank (camera + 2 continuous rotation servos) | `s3cam-rover-tank` | [s3cam-rover-tank.cpp](s3cam-rover-tank.cpp) |
| CYD "Game Boy" controller (display + PCF8574 buttons) | `cyd-rover-controller` | [../cyd-rover-controller/cyd-rover-controller.cpp](../cyd-rover-controller/cyd-rover-controller.cpp) |

Shared packet definitions: [../common/rover_proto.h](../common/rover_proto.h).

## Build and binaries

```bash
pio run -e s3cam-rover-tank
pio run -e cyd-rover-controller
```

[save_bin.py](../../save_bin.py) (post build step of both envs) writes a merged, flashable image to the repo root as `<env>_<YYYYmmdd-HHMMSS>.bin` and deletes the older one of the same env. Flash at offset 0x0:

```bash
esptool.py write_flash 0x0 <env>_<timestamp>.bin
```

Project rule: the newest binary of both envs is always committed and pushed to GitHub (`origin` = flow1986/ESPNowCam).

## Hardware

### CYD (ESP32-2432S028R, hardware of https://github.com/flow1986/cyd-gb)
- PCF8574 at I2C 0x20, SDA = GPIO16, SCL = GPIO17.
- PCF bits: 0 UP, 1 DOWN, 2 LEFT, 3 RIGHT, 4 A, 5 B, 6 START, 7 SELECT (pressed = low).
- Display ILI9341 (TFT_eSPI flags in `platformio.ini`), inverted, rotation 2 (portrait). The camera image is rotated 90 degrees to fill the screen (`-D VIEW_ROTATE=0` for landscape).

### Tank (Freenove ESP32-S3 WROOM CAM pinout, `CamFreenove` driver, `dio_opi`)
- Servos: left GPIO47, right GPIO21 (build flags `SERVO_L_PIN`, `SERVO_R_PIN`).
- Right servo is inverted (mirrored mounting): `SERVO_L_INVERT`, `SERVO_R_INVERT`.
- Trim: `SERVO_L_CENTER_US`, `SERVO_R_CENTER_US` (neutral pulse, default 1500), `SERVO_RANGE_US` (default 400).
- Channel DIP switch (3 switches, each between GPIO and GND, internal pull-ups): bit0 GPIO1, bit1 GPIO2, bit2 GPIO14 (`CH_PIN0..2`). ON = 1. Value 1..7 = tank id = ESP-NOW channel; 000 falls back to 1. GPIO1/2/14 are not strapping pins.
- Weapon output: `FIRE_PIN` (default -1 = none). Set e.g. `-D FIRE_PIN=41` for a laser/IR emitter; it is high while START is held on the CYD and off on failsafe.
- Failsafe: no command for 500 ms stops the servos and the fire output.
- Other S3 boards: change the camera driver and the memory type in `platformio.ini`.

## Controls (CYD)

| Button | Function |
|---|---|
| A / B | forward / backward |
| LEFT / RIGHT | turn |
| UP / DOWN | speed +10% / -10% (default 50%) |
| SELECT | speed back to 50% |
| START (held) | fire (prepared: `fire` flag in `RvDrive`, tank `setFire()` / `FIRE_PIN`) |
| START + SELECT | open / close the config menu |

The HUD (top left) shows tank id and speed, green = tank online, red = offline.

## Config menu

UP/DOWN select, LEFT/RIGHT change (hold = repeat), A runs "Scan"/"Close", B closes. The upper part of the screen shows the live image while adjusting. Settings are stored in NVS when the menu is closed.

- Tank: 1..7, switches the CYD to that channel and addresses only that tank.
- Scan tanks: pings channels 1..7 and jumps to the first tank found.
- Resolution 160x120 / 240x176 / 320x240, JPEG quality 5..50 (higher = bigger frames), brightness / contrast / saturation -2..2, mirror, flip.
- Crosshair on/off (center of the screen, for the later shooting version).

The camera settings are sent to the tank on every change and once per second, so a rebooted tank gets them again.

## Protocol

- Every tank has its own ESP-NOW channel (= DIP value), so several tank/CYD pairs do not share airtime. Packets also carry the tank id.
- Video: JPEG chunks via ESPNowCam (data starts with 0xFF). Control packets start with `RV_MAGIC` 0xA5: `RV_DRIVE`, `RV_CAM`, `RV_PING`, `RV_PONG`.
- ESPNowCam callbacks run in the WiFi task: the CYD only copies the frame there, the tank only sets flags (pong reply and sensor changes happen in `loop()`). `sendData()` uses global state, so the CYD guards it with a mutex.
- Both sides broadcast, no MAC addresses to configure. Two CYDs on the same tank id would both control it.

## Notes / open points

- Not tested on hardware yet, only compiled.
- Start/fire: pressing START slightly before SELECT sends one fire packet before the menu opens.
- Battle mode later: IR emitter on `FIRE_PIN` with a receiver/hit detection on the other tanks, hit reports could use a new `RvType`.
- Display colors (`invertDisplay(true)`) were taken from cyd-gb, adjust in `setup()` if they look wrong.
- Higher quality at 320x240 can exceed the CYD frame buffers (frames are then dropped): `fb` 40000 bytes, `jpg` 32000 bytes.
