# Rover / Tank: CYD controller + ESP32-S3 CAM

Two projects that work together (ESP-NOW, or WiFi / WLAN with a browser interface, see "WiFi mode"):

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
- Channel DIP switch (3 switches, each between GPIO and GND, internal pull-ups): bit0 GPIO1, bit1 GPIO2, bit2 GPIO14 (`CH_PIN0..2`). ON = 1. Value 1..7 = ESP-NOW mode, tank id = ESP-NOW channel; 000 (all off, also when no switch is fitted) = WiFi mode, see "WiFi mode". GPIO1/2/14 are not strapping pins.
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

Start long press enters the mode (HUD: `T1 CAL TRIM +1.2%`, orange). Drive with UP / DOWN (speed with SELECT), LEFT / RIGHT no longer turn but change the trim: if the tank veers to the left press RIGHT, and the other way round, until it drives straight forward and backward. The trim is a balance between the left and right side (left speed x (1 + trim), right speed x (1 - trim)), sent to the tank and stored there. START long press leaves the mode and stores the value. The "Drive trim" menu item shows and edits the same value (reset = bring it to +0.0%).

## Config menu

UP/DOWN select, LEFT/RIGHT change (hold = repeat), A runs "Scan"/"Close", B closes. The upper part of the screen shows the live image while adjusting. Settings are stored in NVS when the menu is closed.

- Tank: 1..7, switches the CYD to that channel and addresses only that tank.
- Scan tanks: ESP-NOW: pings channels 1..7, WiFi: asks mDNS for rovertank1..7; jumps to the first tank found.
- Network mode (ESP-NOW / WiFi, the CYD restarts when the menu is closed) and WiFi setup, see "WiFi mode".
- Resolution 160x120 / 240x176 / 320x240, JPEG quality 5..50 (higher = better quality and larger frames, default 40), brightness / contrast / saturation -2..2, mirror, flip.
- Color and latency tuning: JPEG quality and resolution mostly decide the latency, the other items only change the look. White balance (Auto / Sunny / Cloudy / Office / Home), Effect (None, Negative, Grayscale, Red, Green, Blue, Sepia - keep it on None), Exposure level -2..2, plus brightness / contrast / saturation. The menu status row and the HUD (second line) show the measured `fps` and the average frame size in KB at the display, so quality and latency can be balanced by eye. The color items are stored under their own NVS key.
- Crosshair on/off (center of the screen, for the later shooting version).
- Servo calibration (stored on the tank, edited here or in the browser):
  - Servo L / R center (1000..2000 us, 5 us steps, 20 us while held): while one of these items is selected the tank holds both drive servos at their neutral pulse. Standing still alone does not make the tank drive straight (servos have a dead band and different speeds), use the calibration mode with the drive trim for that.
  - Servo L reverse / Servo R reverse: invert the direction of each drive servo separately (default: left off, right on). If the tank turns on the spot when you drive forward, one of them is wrong; if it drives backward, switch both.
  - Drive trim (-40.0..+40.0%, 0.5% steps): left/right balance, normally set in the calibration mode.
  - Grip min / max and Lift min / max (500..2500 us, 10 us steps, 40 us while held): while an item is selected the tank moves that servo to this pulse, so the end positions can be set by eye. Grip reverse / Lift reverse invert the direction.
- The menu scrolls (28 items).
- Image rotation: only the camera image is rotated, the screen and menu stay portrait. 90 / 270 fill the screen, 0 / 180 are wider than the screen, so the sides are cropped (black bars above and below). Stored on the CYD (own NVS key, independent of the config layout), not per tank.
- The settings layout changed with the calibration: a config saved by an older firmware is ignored once and the defaults are used. The drive trim and the image rotation are stored under their own NVS keys.
- Colors: the sensor JPEG quality is the inverse of the menu value (sensor = 63 - menu). The former default 15 meant a very low quality (grey, washed out colors); the default is now 40 (a one-time migration sets it), a higher value gives better colors but larger frames and more latency.

Camera and servo settings are stored on the tank (NVS, written 2 s after the last change). The menu loads them from the tank when it opens and sends only changes (repeated for 1.5 s to survive lost ESP-NOW packets), so CYD and browser always show the same values. Only the tank choice, image rotation, crosshair, network mode and WLAN credentials are stored on the CYD.

## Protocol

- Every tank has its own ESP-NOW channel (= DIP value), so several tank/CYD pairs do not share airtime. Packets also carry the tank id.
- Video: the camera sensor produces JPEG directly; JPEG chunks go via ESPNowCam (data starts with 0xFF). This avoids the former CPU-side RGB565-to-JPEG conversion. The serial monitor reports `TX fps`, average send time and average frame size every two seconds. Control packets start with `RV_MAGIC` 0xA5: `RV_DRIVE` (drive, fire, gripper and lift positions), `RV_CAM`, `RV_SERVO` (settings: set by a remote, stored on the tank, also the reply to `RV_GETCFG`; the preview field is never stored), `RV_GETCFG`, `RV_PING`, `RV_PONG` (with the gripper / lift position, so a remote continues where the last one stopped).
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

## WiFi mode (WLAN)

Implemented: the system runs either with ESP-NOW or in a normal WLAN, with several tanks and CYDs in one network and a browser interface.

- Mode: tank DIP switch 0 (all off, also when no switch is fitted) = WiFi mode, 1..7 = ESP-NOW with tank id = channel. CYD: menu "Network mode" (ESP-NOW / WiFi), applied by a restart when the menu is closed.
- Tank setup: at the first start (no WLAN saved) or when the WLAN is not reachable within 15 s the tank opens the access point `RoverTank-Setup` (password `rovertank`, build flags `AP_SSID` / `AP_PASSWORD`). Config page: http://192.168.4.1/wifi (SSID list, password, tank id 1..7, must be unique). It is saved in NVS, the tank restarts and joins the WLAN. The page stays available at `/wifi` in the WLAN. The control page also works in the access point (http://192.168.4.1/), that is a direct connection without router.
- Address of the tank: http://rovertank<id>.local (mDNS) or the IP from the serial monitor (115200 baud). Some Android browsers cannot resolve .local, use the IP then.
- Web UI (phone and PC): live video (rotation and crosshair are browser settings), touch buttons for drive, gripper, lift, speed 25/50/75/100 and FIRE, and a settings panel with all camera and servo settings including the preview on the tank (stored on the tank). Keyboard: W/S or arrow up/down forward/backward, A/D or arrow left/right turn, Q/E grip open/close, R/F lift up/down, Space fire, 1..4 speed 25..100%, +/- speed.
- CYD in WiFi mode: menu "WiFi setup" scans the networks (UP/DOWN + A) and asks for the password with a character editor (UP/DOWN character, RIGHT next / add, LEFT back, B delete, A ok, SELECT cancel). It works after the mode was switched to WiFi and the CYD restarted. "Scan tanks" asks mDNS for rovertank1..7, "Tank" chooses the id, then the CYD opens a WebSocket to the tank.
- Settings on the tank: camera and servo settings live in the NVS of the tank. CYD and browser read them with `RV_GETCFG` when they connect (CYD: when the menu opens) and send only changes, so switching between CYD and browser is safe. The CYD only sends its working copy after it has received the tank values, so it can never overwrite them with defaults.
- One remote per tank: there is no lock. If a CYD and a browser send at the same time the last packet wins. The 500 ms failsafe stays.
- Video over WebSocket: a JPEG is sent as messages `[0xFE][flags: 1 first, 2 last][max 12000 bytes]` (the WebSocket library rejects frames above 15 KB). The tank drops frames when the network task is still busy or nobody is connected.
- Libraries: links2004/WebSockets 2.4.1 (server on port 81 on the tank, client on the CYD), WebServer, ESPmDNS. The CYD allocates its frame buffers (40000 + 32000 bytes) at start because the static DRAM is too small.
- Security: there is no authentication yet. Anybody in the WLAN can open the page and drive the tank, WPA2 of the WLAN is the only protection (access point: change `AP_PASSWORD`). A token / password for the WebSocket would be the next step.
- Not tested on hardware, only compiled: mDNS lookup on the CYD, WebSocket throughput, the web page in real browsers.

## WiFi mode: original planning notes (implemented above, kept for reference)

Goal: run the system either with ESP-NOW (as now) or in a normal WLAN, several tanks and CYDs in one network, plus a browser interface.

- Mode select: the first idea was DIP 0..6 = ESP-NOW and 7 = WiFi, it was changed to DIP 0 = WiFi and 1..7 = ESP-NOW (no id shift, and a missing switch gives WiFi). A change needs a reboot. One firmware per device contains both modes, the CYD selects the mode in its menu.
- Setup: in WiFi mode the tank first opens an AP with a config page (SSID, password, tank id / name), stores it in NVS and then joins the existing WLAN. If joining fails it falls back to the AP. The CYD can join the WLAN from its menu. Typing a password with the D-pad is tedious: add a small on-screen character picker or an AP config page on the CYD as well.
- "Channels" in the WLAN are only tank ids. The CYD finds tanks by UDP broadcast ping/pong (the existing `RV_PING` / `RV_PONG`) or mDNS (`tank-3.local`) and lists them in the menu like the current scan. Ids must be unique.
- Transport: the same packets as now (`RV_DRIVE`, `RV_CAM`, `RV_SERVO`, ping) over WebSocket or UDP instead of ESP-NOW, so the tank code mainly changes at the transport. The tank serves a web page: MJPEG video, drive / gripper / camera controls and all settings of the CYD menu. Browser and CYD are both controllers.
- Settings on the tank: calibration, trim and camera values have to move into the tank NVS (today per tank id on the CYD), because browser and CYD need the same values.
- Several controllers: only one is active per tank (lock or last one wins), the 500 ms failsafe stays.
- Security: protect the web interface (WPA2, password or token), otherwise anybody in the WLAN can drive the tank.
- Bandwidth: WiFi gives far more fps than ESP-NOW (about 1 fps measured now). One stream per tank to one CYD plus one browser is realistic. 5 to 6 tanks streaming at the same time can be too much for one 2.4 GHz WLAN, then lower resolution / quality. A slow client must not stall the tank: drop old frames instead of queueing.
- Latency: the path tank - router - CYD has two WiFi hops (a few to about 30 ms more, jitter 50 to 200 ms on a busy WLAN), but the much faster frame transfer should give a lower total latency than ESP-NOW now. The fixed parts stay: camera exposure + JPEG on the tank and decode + draw on the CYD. Keep WiFi power save off on both sides, keep one frame in flight, choose a free channel, place the router near the playground.
- Lowest latency alternative: the CYD joins the AP of the tank directly (one hop, no router), but then only a few clients per tank and no shared network.
- The CYD can do normal WLAN. The earlier doubt was about the raw 802.11 mode of the library, which is something different.
- Order: 1. tank with WiFi mode, AP config and web interface, 2. CYD as WiFi client with tank search, 3. shared settings on the tank.

## Notes / open points

- Menu: START + SELECT opens/closes it (B or A on "Close" also close it).
- Not tested on hardware yet, only compiled.
- Start/fire: pressing START slightly before SELECT sends one fire packet before the menu opens.
- Battle mode later: IR emitter on `FIRE_PIN` with a receiver/hit detection on the other tanks, hit reports could use a new `RvType`.
- Display colors (`invertDisplay(true)`) were taken from cyd-gb, adjust in `setup()` if they look wrong.
- Higher quality at 320x240 can exceed the CYD frame buffers (frames are then dropped): `fb` 40000 bytes, `jpg` 32000 bytes.
