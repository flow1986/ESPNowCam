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
- Display ILI9341 (TFT_eSPI flags in `platformio.ini`), inverted, always portrait (the menu keeps its orientation). The rotation of the camera image (0/90/180/270 degrees) is a menu setting, default 90 (image fills the screen).

### Tank (Freenove ESP32-S3 WROOM CAM pinout, `CamFreenove` driver, `dio_opi`)
- Servos: left GPIO47, right GPIO21 (build flags `SERVO_L_PIN`, `SERVO_R_PIN`).
- Right servo is inverted by default (mirrored mounting); "Servo L reverse" / "Servo R reverse" in the CYD menu change this per servo (the old `SERVO_*_INVERT` build flags are gone).
- Neutral pulse (center) and the drive trim of the drive servos are calibrated on the CYD (see below), `SERVO_RANGE_US` (default 400) is the pulse offset at 100% speed.
- Gripper (normal positional servos): grip servo GPIO41, lift servo GPIO42 (`GRIP_PIN`, `LIFT_PIN`). Verify these pins against your board pinout. Min/max pulse and reverse are set in the CYD menu.
- The camera clock uses LEDC timer 3 / channel 7, the servos only get timers 0..2, so the PWM of the servos cannot disturb the camera.
- Channel DIP switch (3 switches, each between GPIO and GND, internal pull-ups): bit0 GPIO1, bit1 GPIO2, bit2 GPIO14 (`CH_PIN0..2`). ON = 1. Value 1..7 = tank id = ESP-NOW channel; 000 falls back to 1. GPIO1/2/14 are not strapping pins.
- Weapon output: `FIRE_PIN` (default -1 = none). Set e.g. `-D FIRE_PIN=40` for a laser/IR emitter (41/42 are used by the gripper); it is high while START is held on the CYD and off on failsafe.
- Failsafe: no command for 500 ms stops the servos and the fire output.
- Other S3 boards: change the camera driver and the memory type in `platformio.ini`.

## Controls (CYD)

| Button | Function |
|---|---|
| UP / DOWN | forward / backward |
| LEFT / RIGHT | turn |
| SELECT (short) | speed cycles 25% - 50% - 75% - 100% (default 50%) |
| A / B | gripper close / open |
| SELECT held + A / B | lift up / down |
| START (held) | fire (prepared: `fire` flag in `RvDrive`, tank `setFire()` / `FIRE_PIN`), only for the first 0.8 s of the press |
| START (long, 0.8 s) | enter / leave the calibration mode |
| START + SELECT | open / close the config menu |

The HUD (top left) shows `T1 SPD 50% G50 L50` (tank id, speed, gripper and lift position in percent), green = tank online, red = offline. The gripper moves with 60%/s while a key is held (`GRIP_RATE_PCT_S`), the CYD sends the absolute positions, the tank holds the last position if the link is lost. 0% = min pulse, 100% = max pulse; if a direction is wrong, switch "Grip reverse" / "Lift reverse".

### Calibration mode (drive straight)

Start long press enters the mode (HUD: `T1 CAL TRIM +1.2%`, orange). Drive with UP / DOWN (speed with SELECT), LEFT / RIGHT no longer turn but change the trim: if the tank veers to the left press RIGHT, and the other way round, until it drives straight forward and backward. The trim is a balance between the left and right side (left speed x (1 + trim), right speed x (1 - trim)), stored per tank id on the CYD and sent to the tank. START long press leaves the mode and stores the value. The "Drive trim" menu item shows and edits the same value (reset = bring it to +0.0%).

## Config menu

UP/DOWN select, LEFT/RIGHT change (hold = repeat), A runs "Scan"/"Close", B closes. The upper part of the screen shows the live image while adjusting. Settings are stored in NVS when the menu is closed.

- Tank: 1..7, switches the CYD to that channel and addresses only that tank.
- Scan tanks: pings channels 1..7 and jumps to the first tank found.
- Resolution 160x120 / 240x176 / 320x240, JPEG quality 5..50 (higher = better quality and larger frames, default 40), brightness / contrast / saturation -2..2, mirror, flip.
- Crosshair on/off (center of the screen, for the later shooting version).
- Servo calibration (stored per tank id on the CYD, sent to the tank every second):
  - Servo L / R center (1000..2000 us, 5 us steps, 20 us while held): while one of these items is selected the tank holds both drive servos at their neutral pulse. Standing still alone does not make the tank drive straight (servos have a dead band and different speeds), use the calibration mode with the drive trim for that.
  - Servo L reverse / Servo R reverse: invert the direction of each drive servo separately (default: left off, right on). If the tank turns on the spot when you drive forward, one of them is wrong; if it drives backward, switch both.
  - Drive trim (-40.0..+40.0%, 0.5% steps): left/right balance, normally set in the calibration mode.
  - Grip min / max and Lift min / max (500..2500 us, 10 us steps, 40 us while held): while an item is selected the tank moves that servo to this pulse, so the end positions can be set by eye. Grip reverse / Lift reverse invert the direction.
- The menu scrolls (23 items).
- Image rotation: only the camera image is rotated, the screen and menu stay portrait. 90 / 270 fill the screen, 0 / 180 are wider than the screen, so the sides are cropped (black bars above and below). Stored on the CYD (own NVS key, independent of the config layout), not per tank.
- The settings layout changed with the calibration: a config saved by an older firmware is ignored once and the defaults are used. The drive trim and the image rotation are stored under their own NVS keys.
- Colors: the sensor JPEG quality is the inverse of the menu value (sensor = 63 - menu). The former default 15 meant a very low quality (grey, washed out colors); the default is now 40 (a one-time migration sets it), a higher value gives better colors but larger frames and more latency.

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
