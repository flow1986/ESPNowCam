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

For a webflasher that accepts separate files and offsets, a S3 build also writes four timestamp-matched files to the repo root:

| File suffix | Flash offset |
|---|---:|
| `_bootloader.bin` | `0x0` |
| `_partitions.bin` | `0x8000` |
| `_boot_app0.bin` | `0xE000` |
| `_firmware.bin` | `0x10000` |

Select all four files from the same timestamp in the webflasher and enter the offsets above. If the webflasher accepts one merged image instead, select `<env>_<timestamp>.bin` at offset `0x0`. The CYD build continues to produce its single merged image.

Project rule: the newest binary of both envs is always committed and pushed to GitHub (`origin` = flow1986/ESPNowCam).

## Hardware

### CYD (ESP32-2432S028R, hardware of https://github.com/flow1986/cyd-gb)
- PCF8574 at I2C 0x20, SDA = GPIO16, SCL = GPIO17.
- PCF bits: 0 UP, 1 DOWN, 2 LEFT, 3 RIGHT, 4 A, 5 B, 6 START, 7 SELECT (pressed = low).
- Display ILI9341 (TFT_eSPI flags in `platformio.ini`), inverted. The camera view rotation (0/90/180/270 degrees) is a menu setting, default 90 (image rotated to fill the portrait screen).

### Tank (Freenove ESP32-S3 WROOM CAM pinout, `CamFreenove` driver, `dio_opi`)
- Servos: left GPIO47, right GPIO21 (build flags `SERVO_L_PIN`, `SERVO_R_PIN`).
- Right servo is inverted by default (mirrored mounting); "Servo L reverse" / "Servo R reverse" in the CYD menu change this per servo (the old `SERVO_*_INVERT` build flags are gone).
- Neutral pulse (center) of the drive servos is calibrated in the CYD menu (see below), `SERVO_RANGE_US` (default 400) is the pulse offset at 100% speed.
- Gripper (normal positional servos): grip servo GPIO41, lift servo GPIO42 (`GRIP_PIN`, `LIFT_PIN`). Verify these pins against your board pinout. Min/max pulse and reverse are set in the CYD menu.
- The camera clock uses LEDC timer 3 / channel 7, the servos only get timers 0..2, so the PWM of the servos cannot disturb the camera.
- Channel DIP switch (3 switches, each between GPIO and GND, internal pull-ups): bit0 GPIO1, bit1 GPIO2, bit2 GPIO14 (`CH_PIN0..2`). ON = 1. Value 1..7 = tank id = ESP-NOW channel; 000 falls back to 1. GPIO1/2/14 are not strapping pins.
- Weapon output: `FIRE_PIN` (default -1 = none). Set e.g. `-D FIRE_PIN=40` for a laser/IR emitter (41/42 are used by the gripper); it is high while START is held on the CYD and off on failsafe.
- Failsafe: no command for 500 ms stops the servos and the fire output.
- Other S3 boards: change the camera driver and the memory type in `platformio.ini`.

## Controls (CYD)

| Button | Function |
|---|---|
| SELECT (short) | switch between drive mode and gripper mode (shown in the HUD) |
| SELECT (long, 0.6 s) | speed back to 50% |
| UP / DOWN | drive mode: speed +10% / -10% (default 50%). Gripper mode: lift up / down |
| A / B | forward / backward, both modes |
| LEFT / RIGHT | drive mode: turn. Gripper mode: gripper open / close |
| START (held) | fire (prepared: `fire` flag in `RvDrive`, tank `setFire()` / `FIRE_PIN`) |
| START + SELECT | open / close the config menu |

The HUD (top left) shows tank id and mode: `T1 DRIVE SPD 50%` or `T1 GRIP G50 L50` (gripper and lift position in percent). Green/yellow = tank online, red = offline.

In gripper mode the tank drives forward/backward with A/B at the current speed, but cannot turn and the speed cannot be changed (switch back to drive mode for that). The gripper moves with 60%/s while a key is held (`GRIP_RATE_PCT_S`), the CYD sends the absolute positions, the tank holds the last position if the link is lost. 0% = min pulse, 100% = max pulse; if a direction is wrong, switch "Grip reverse" / "Lift reverse".

## Config menu

UP/DOWN select, LEFT/RIGHT change (hold = repeat), A runs "Scan"/"Close", B closes. The upper part of the screen shows the live image while adjusting. Settings are stored in NVS when the menu is closed.

- Tank: 1..7, switches the CYD to that channel and addresses only that tank.
- Scan tanks: pings channels 1..7 and jumps to the first tank found.
- Resolution 160x120 / 240x176 / 320x240, JPEG quality 5..50 (higher = better quality and larger frames), brightness / contrast / saturation -2..2, mirror, flip.
- Crosshair on/off (center of the screen, for the later shooting version).
- Servo calibration (stored per tank id on the CYD, sent to the tank every second):
  - Servo L / R center (1000..2000 us, 5 us steps, 20 us while held): while one of these items is selected the tank holds both drive servos at their neutral pulse, adjust until they stand still. For continuous rotation servos the center is the important value.
  - Servo L reverse / Servo R reverse: invert the direction of each drive servo separately (default: left off, right on). If the tank turns on the spot when you drive forward, one of them is wrong; if it drives backward, switch both.
  - Grip min / max and Lift min / max (500..2500 us, 10 us steps, 40 us while held): while an item is selected the tank moves that servo to this pulse, so the end positions can be set by eye. Grip reverse / Lift reverse invert the direction.
- The menu scrolls (22 items).
- Rotation: 0 / 180 = landscape view, 90 / 270 = portrait view filling the screen. Stored on the CYD (own NVS key, independent of the config layout), not per tank.
- The settings layout changed with the calibration: a config saved by an older firmware is ignored once and the defaults are used.

The camera settings are sent to the tank on every change and once per second, so a rebooted tank gets them again.

## Protocol

- Every tank has its own ESP-NOW channel (= DIP value), so several tank/CYD pairs do not share airtime. Packets also carry the tank id.
- Video: the camera sensor produces JPEG directly; JPEG chunks go via ESPNowCam (data starts with 0xFF). This avoids the former CPU-side RGB565-to-JPEG conversion. The serial monitor reports `TX fps`, average send time and average frame size every two seconds. Control packets start with `RV_MAGIC` 0xA5: `RV_DRIVE` (drive, fire, gripper and lift positions), `RV_CAM`, `RV_SERVO` (calibration and preview), `RV_PING`, `RV_PONG`.
- ESPNowCam callbacks run in the WiFi task: the CYD only copies the frame there, the tank only sets flags (pong reply and sensor changes happen in `loop()`). `sendData()` uses global state, so the CYD guards it with a mutex.
- Both sides broadcast, no MAC addresses to configure. Two CYDs on the same tank id would both control it.

## Combat plan (not implemented yet, only `FIRE_PIN` / `fire` flag are prepared)

Decision: modulated IR (38 kHz) with a small code containing the shooter id, instead of laser pointer + photodiode.

- Why IR: IR receiver modules (TSOP38238 / VS1838B) reject ambient light and sunlight, the code can carry the tank id, and it is eye-safe. Laser + raw photodiode is sensitive to light, needs modulation anyway, carries no id and is a laser class 2 eye risk.
- Transmitter: 940 nm LED with a narrow angle (e.g. TSAL6200, about +-5 degrees) switched by a transistor/MOSFET (a GPIO is too weak), 38 kHz carrier via LEDC or IRremote (ESP32-S3 compatibility still to be checked). A black tube / heat shrink (5-10 cm) in front narrows the beam. Mount it next to the camera, parallel to the view axis, so the crosshair matches.
- Receivers: one module each at front, left, right and back, each with a small shroud. Hit zones allow different damage (e.g. more from behind).
- Game logic: tank detects a hit and reports it via ESP-NOW (new `RvType`, e.g. `RV_HIT`) to its own CYD, which shows lives/HUD/hit effect. Hit = tank disabled for about 2 s. Fire rate limit (about 1 shot per 0.5-1 s) on the tank.
- Limitation: every tank/CYD pair has its own channel, so hits and lives are local per tank. A shared scoreboard would need a channel bridge or a referee, later.
- Order: 1 tank with emitter + receiver and hit display on the CYD, then a second tank, then hit zones and rules.

## Notes / open points

- Menu: START + SELECT opens/closes it (B or A on "Close" also close it).
- Not tested on hardware yet, only compiled.
- Start/fire: pressing START slightly before SELECT sends one fire packet before the menu opens.
- Battle mode later: IR emitter on `FIRE_PIN` with a receiver/hit detection on the other tanks, hit reports could use a new `RvType`.
- Display colors (`invertDisplay(true)`) were taken from cyd-gb, adjust in `setup()` if they look wrong.
- Higher quality at 320x240 can exceed the CYD frame buffers (frames are then dropped): `fb` 40000 bytes, `jpg` 32000 bytes.
