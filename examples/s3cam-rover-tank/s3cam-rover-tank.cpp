/**************************************************
 * ESP32-S3 CAM Rover (Freenove pinout)
 * Streams the camera via ESP-NOW and drives two continuous rotation
 * servos (tank) plus a gripper (grip + lift servo) from the commands
 * of the CYD rover controller.
 * Tank id (1..7) = ESP-NOW channel, read from a 3 pin DIP switch,
 * no switch fitted (all pins open) = id 1.
 * Camera and servo settings are stored on the tank (NVS), the CYD
 * reads them with RV_GETCFG and sends only changes.
 * This file is part ESPNowCam project:
 * https://github.com/hpsaturn/ESPNowCam
**************************************************/

#include <Arduino.h>
#include <Preferences.h>
#include <esp_wifi.h>
#include <ESPNowCam.h>
#include <common/rover_proto.h>
#include <drivers/CamFreenove.h>
#include <ESP32Servo.h>

#ifndef SERVO_L_PIN
#define SERVO_L_PIN 47
#endif
#ifndef SERVO_R_PIN
#define SERVO_R_PIN 21
#endif
// pulse width offset at 100% speed (the neutral pulse is calibrated on the CYD)
#ifndef SERVO_RANGE_US
#define SERVO_RANGE_US 400
#endif
// gripper servos (normal positional servos)
#ifndef GRIP_PIN
#define GRIP_PIN 41
#endif
#ifndef LIFT_PIN
#define LIFT_PIN 42
#endif
// DIP switch to GND, bit0..bit2 (ON = 1). Value 1..7 is the channel, 0 falls back to 1
#ifndef CH_PIN0
#define CH_PIN0 1
#endif
#ifndef CH_PIN1
#define CH_PIN1 2
#endif
#ifndef CH_PIN2
#define CH_PIN2 14
#endif
// weapon output (laser / IR emitter), -1 = none
#ifndef FIRE_PIN
#define FIRE_PIN -1
#endif
#define CMD_TIMEOUT_MS 500
#define CFG_SAVE_DELAY_MS 2000

CamFreenove Camera;
ESPNowCam radio;
Servo servoLeft;
Servo servoRight;
Servo servoGrip;
Servo servoLift;
Preferences tp;

static uint8_t *recv_buff;
static uint8_t tankId = 1;
static volatile uint32_t lastCmdStamp = 0;
static volatile bool pongPending = false;
static volatile bool cfgReplyPending = false;
static volatile bool camPending = false;

// settings of this tank, stored in NVS (preview is never stored)
static RvCam camCfg = {{RV_MAGIC, RV_CAM, 0}, 2, 40, 0, 0, 0, 0, 0, 0, 0, 0};
static RvCam camApplied;
static bool camAppliedValid = false;
static RvServo sv = {{RV_MAGIC, RV_SERVO, 0}, 1500, 1500, 1000, 2000, 1000, 2000, 0, 0, 0, 1, 0, 0};
static bool cfgDirty = false;
static uint32_t cfgDirtyStamp = 0;

static int gripPct = 50, liftPct = 50;
static bool gripperActive = false;  // after the first drive packet
static int lastGripUs = 0, lastLiftUs = 0;

static const framesize_t frameSizes[] = {FRAMESIZE_QQVGA, FRAMESIZE_HQVGA, FRAMESIZE_QVGA};

/***********************************
 * Settings (NVS)
************************************/
void loadCfg() {
  RvCam c;
  if (tp.getBytes("cam", &c, sizeof(c)) == sizeof(c)) camCfg = c;
  RvServo s;
  if (tp.getBytes("sv", &s, sizeof(s)) == sizeof(s)) sv = s;
  sv.preview = 0;
  camCfg.h = {RV_MAGIC, RV_CAM, tankId};
  sv.h = {RV_MAGIC, RV_SERVO, tankId};
}

void saveCfgIfDue() {
  if (!cfgDirty || millis() - cfgDirtyStamp < CFG_SAVE_DELAY_MS) return;
  cfgDirty = false;
  RvServo s = sv;
  s.preview = 0;
  tp.putBytes("cam", &camCfg, sizeof(camCfg));
  tp.putBytes("sv", &s, sizeof(s));
  Serial.println("Settings saved");
}

/***********************************
 * Servos
************************************/
uint8_t readDip() {
  const uint8_t pins[3] = {CH_PIN0, CH_PIN1, CH_PIN2};
  uint8_t v = 0;
  for (int i = 0; i < 3; i++) {
    pinMode(pins[i], INPUT_PULLUP);
    delay(2);
    if (!digitalRead(pins[i])) v |= 1 << i;
  }
  return (v >= 1 && v <= RV_MAX_TANKS) ? v : 1;
}

void setFire(bool on) {
  if (FIRE_PIN >= 0) digitalWrite(FIRE_PIN, on);
}

// pct: -100..100. At 0 the servo is released so it can't creep, except while calibrating
void driveServo(Servo &servo, int pin, int pct, bool invert, int centerUs) {
  if (pct == 0 && sv.preview != 1) {
    if (servo.attached()) servo.detach();
    return;
  }
  if (!servo.attached()) servo.attach(pin, 500, 2500);
  if (invert) pct = -pct;
  servo.writeMicroseconds(centerUs + pct * SERVO_RANGE_US / 100);
}

void drive(int fwd, int turn) {
  // trim balances the two sides of the tank for driving straight
  int l = constrain(fwd * (1000 + sv.trim) / 1000 + turn, -100, 100);
  int r = constrain(fwd * (1000 - sv.trim) / 1000 - turn, -100, 100);
  driveServo(servoLeft, SERVO_L_PIN, l, sv.revL, sv.centerL);
  driveServo(servoRight, SERVO_R_PIN, r, sv.revR, sv.centerR);
}

int posToUs(int pct, int minUs, int maxUs, bool rev) {
  if (rev) pct = 100 - pct;
  return constrain(minUs + (maxUs - minUs) * pct / 100, 500, 2500);
}

void writeGripperServo(Servo &servo, int pin, int us, int &lastUs) {
  if (!servo.attached()) servo.attach(pin, 500, 2500);
  if (us != lastUs) {
    servo.writeMicroseconds(us);
    lastUs = us;
  }
}

void updateGripper() {
  if (!gripperActive && sv.preview < 2) return;
  int g = posToUs(gripPct, sv.gripMin, sv.gripMax, sv.gripRev);
  int l = posToUs(liftPct, sv.liftMin, sv.liftMax, sv.liftRev);
  if (sv.preview == 2) g = constrain(sv.gripMin, 500, 2500);
  if (sv.preview == 3) g = constrain(sv.gripMax, 500, 2500);
  if (sv.preview == 4) l = constrain(sv.liftMin, 500, 2500);
  if (sv.preview == 5) l = constrain(sv.liftMax, 500, 2500);
  writeGripperServo(servoGrip, GRIP_PIN, g, lastGripUs);
  writeGripperServo(servoLift, LIFT_PIN, l, lastLiftUs);
}

/***********************************
 * Packets
************************************/
void onCamPacket(const uint8_t *d) {
  RvCam c;
  memcpy(&c, d, sizeof(c));
  if (memcmp((const uint8_t *)&c + sizeof(RvHdr), (const uint8_t *)&camCfg + sizeof(RvHdr), sizeof(RvCam) - sizeof(RvHdr)) != 0) {
    camCfg = c;
    camCfg.h = {RV_MAGIC, RV_CAM, tankId};
    camPending = true;
    cfgDirty = true;
    cfgDirtyStamp = millis();
  }
}

void onServoPacket(const uint8_t *d) {
  RvServo s;
  memcpy(&s, d, sizeof(s));
  uint8_t oldPreview = sv.preview;
  // everything after the header except the preview byte
  bool changed = memcmp((const uint8_t *)&s + sizeof(RvHdr), (const uint8_t *)&sv + sizeof(RvHdr),
                        sizeof(RvServo) - sizeof(RvHdr) - 1) != 0;
  sv = s;
  sv.h = {RV_MAGIC, RV_SERVO, tankId};
  if (changed) {
    cfgDirty = true;
    cfgDirtyStamp = millis();
  }
  // the CYD sends zero drive commands while calibrating, so this can't interrupt driving
  if (sv.preview == 1 || oldPreview == 1) drive(0, 0);
  updateGripper();
}

// runs in the WiFi task: no radio sends and no sensor access here
void onDataReady(uint32_t length) {
  if (length < sizeof(RvHdr)) return;
  const RvHdr *h = (const RvHdr *)recv_buff;
  if (h->magic != RV_MAGIC) return;
  bool mine = (h->tank == tankId);
  bool any = (h->tank == 0 || mine);

  if (h->type == RV_PING && any) {
    pongPending = true;
  } else if (h->type == RV_GETCFG && any) {
    cfgReplyPending = true;
  } else if (mine) {
    if (h->type == RV_DRIVE && length >= sizeof(RvDrive)) {
      const RvDrive *d = (const RvDrive *)recv_buff;
      lastCmdStamp = millis();
      drive(d->fwd, d->turn);
      setFire(d->fire);
      gripPct = constrain(d->grip, 0, 100);
      liftPct = constrain(d->lift, 0, 100);
      gripperActive = true;
      updateGripper();
    } else if (h->type == RV_CAM && length >= sizeof(RvCam)) {
      onCamPacket(recv_buff);
    } else if (h->type == RV_SERVO && length >= sizeof(RvServo)) {
      onServoPacket(recv_buff);
    }
  }
}

// replies are sent from loop(), the radio is not used from the callback
void serviceReplies() {
  if (pongPending) {
    pongPending = false;
    RvPong r = {{RV_MAGIC, RV_PONG, tankId}, (uint8_t)gripPct, (uint8_t)liftPct};
    radio.sendData((uint8_t *)&r, sizeof(r));
  }
  if (cfgReplyPending) {
    cfgReplyPending = false;
    RvCam c = camCfg;
    radio.sendData((uint8_t *)&c, sizeof(c));
    RvServo s = sv;
    s.preview = 0;
    radio.sendData((uint8_t *)&s, sizeof(s));
  }
}

/***********************************
 * Camera
************************************/
void applyCam() {
  RvCam c = camCfg;
  camPending = false;
  sensor_t *s = Camera.sensor;
  if (!s) return;
  bool first = !camAppliedValid;
  if (c.size < 3 && (first || c.size != camApplied.size)) s->set_framesize(s, frameSizes[c.size]);
  if (first || c.brightness != camApplied.brightness) s->set_brightness(s, constrain(c.brightness, -2, 2));
  if (first || c.contrast != camApplied.contrast) s->set_contrast(s, constrain(c.contrast, -2, 2));
  if (first || c.saturation != camApplied.saturation) s->set_saturation(s, constrain(c.saturation, -2, 2));
  // sensor quality is inverted (lower = better), the menu value is not
  if (first || c.quality != camApplied.quality) s->set_quality(s, 63 - constrain(c.quality, 5, 50));
  if (first || c.hmirror != camApplied.hmirror) s->set_hmirror(s, c.hmirror);
  if (first || c.vflip != camApplied.vflip) s->set_vflip(s, c.vflip);
  if (first || c.wb != camApplied.wb) {
    s->set_whitebal(s, 1);
    s->set_wb_mode(s, constrain(c.wb, 0, 4));
  }
  if (first || c.effect != camApplied.effect) s->set_special_effect(s, constrain(c.effect, 0, 6));
  if (first || c.aeLevel != camApplied.aeLevel) s->set_ae_level(s, constrain(c.aeLevel, -2, 2));
  camApplied = c;
  camAppliedValid = true;
}

void processFrame() {
  if (!Camera.get()) return;
  if (Camera.fb->format != PIXFORMAT_JPEG) {
    Camera.free();
    return;
  }
  static uint32_t windowStart = millis();
  static uint32_t frames = 0;
  static uint32_t totalSendUs = 0;
  static uint32_t totalBytes = 0;
  uint32_t sendStart = micros();
  radio.sendData(Camera.fb->buf, Camera.fb->len);
  totalSendUs += micros() - sendStart;
  totalBytes += Camera.fb->len;
  frames++;
  Camera.free();
  uint32_t elapsed = millis() - windowStart;
  if (elapsed >= 2000) {
    Serial.printf("TX %.1f fps, %.1f ms/frame, %.1f KB/frame\r\n", frames * 1000.0f / elapsed,
                  totalSendUs / 1000.0f / frames, totalBytes / 1024.0f / frames);
    windowStart = millis();
    frames = totalSendUs = totalBytes = 0;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  tp.begin("tank", false);
  tankId = readDip();
  Serial.printf("Tank id / channel: %u\r\n", tankId);
  loadCfg();
  camPending = true;

  if (FIRE_PIN >= 0) {
    pinMode(FIRE_PIN, OUTPUT);
    setFire(false);
  }

  // timer 3 / channel 7 stay reserved for the camera clock, the servos use the others
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  servoLeft.setPeriodHertz(50);
  servoRight.setPeriodHertz(50);
  servoGrip.setPeriodHertz(50);
  servoLift.setPeriodHertz(50);

  Camera.config.ledc_timer = LEDC_TIMER_3;
  Camera.config.ledc_channel = LEDC_CHANNEL_7;
  Camera.config.pixel_format = PIXFORMAT_JPEG;
  Camera.config.jpeg_quality = 15;
  if (Camera.begin()) Serial.println("Camera Init Success (native JPEG)");
  else Serial.println("Camera Init Failed");

  recv_buff = static_cast<uint8_t *>(ps_malloc(100));
  radio.setRecvBuffer(recv_buff);
  radio.setRecvCallback(onDataReady);
  if (radio.init(244)) Serial.println("ESPNow Init Success");
  esp_wifi_set_channel(tankId, WIFI_SECOND_CHAN_NONE);
}

void loop() {
  processFrame();
  serviceReplies();
  if (camPending) applyCam();
  saveCfgIfDue();
  // failsafe: stop driving if the controller is gone, the gripper keeps its position
  if (millis() - lastCmdStamp > CMD_TIMEOUT_MS) {
    drive(0, 0);
    setFire(false);
  }
}
