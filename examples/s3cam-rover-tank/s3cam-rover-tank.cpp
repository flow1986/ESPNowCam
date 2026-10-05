/**************************************************
 * ESP32-S3 CAM Rover (Freenove pinout)
 * Streams the camera via ESP-NOW and drives two continuous rotation
 * servos (tank) from the commands of the CYD rover controller.
 * Tank id (1..7) = ESP-NOW channel, read from a 3 pin DIP switch.
 * This file is part ESPNowCam project:
 * https://github.com/hpsaturn/ESPNowCam
**************************************************/

#include <Arduino.h>
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
// the servos are mirrored on a tank, so one of them must be inverted
#ifndef SERVO_L_INVERT
#define SERVO_L_INVERT 0
#endif
#ifndef SERVO_R_INVERT
#define SERVO_R_INVERT 1
#endif
// neutral pulse where the servo stands still (trim with the servo screw or here)
#ifndef SERVO_L_CENTER_US
#define SERVO_L_CENTER_US 1500
#endif
#ifndef SERVO_R_CENTER_US
#define SERVO_R_CENTER_US 1500
#endif
// pulse width offset at 100% speed
#ifndef SERVO_RANGE_US
#define SERVO_RANGE_US 400
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

CamFreenove Camera;
ESPNowCam radio;
Servo servoLeft;
Servo servoRight;

static uint8_t *recv_buff;
static uint8_t tankId = 1;
static volatile uint32_t lastCmdStamp = 0;
static volatile bool pongPending = false;
static volatile bool camPending = false;
static RvCam camNew;
static RvCam camApplied;
static bool camAppliedValid = false;
static volatile uint8_t jpgQuality = 15;

static const framesize_t frameSizes[] = {FRAMESIZE_QQVGA, FRAMESIZE_HQVGA, FRAMESIZE_QVGA};

uint8_t readTankId() {
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

// pct: -100..100, 0 releases the servo so it can't creep
void driveServo(Servo &servo, int pin, int pct, bool invert, int centerUs) {
  if (pct == 0) {
    if (servo.attached()) servo.detach();
    return;
  }
  if (!servo.attached()) servo.attach(pin, 500, 2500);
  if (invert) pct = -pct;
  servo.writeMicroseconds(centerUs + pct * SERVO_RANGE_US / 100);
}

void drive(int fwd, int turn) {
  int l = constrain(fwd + turn, -100, 100);
  int r = constrain(fwd - turn, -100, 100);
  driveServo(servoLeft, SERVO_L_PIN, l, SERVO_L_INVERT, SERVO_L_CENTER_US);
  driveServo(servoRight, SERVO_R_PIN, r, SERVO_R_INVERT, SERVO_R_CENTER_US);
}

// runs in the WiFi task: no radio sends and no sensor access here
void onDataReady(uint32_t length) {
  if (length < sizeof(RvHdr)) return;
  const RvHdr *h = (const RvHdr *)recv_buff;
  if (h->magic != RV_MAGIC) return;

  if (h->type == RV_PING && (h->tank == 0 || h->tank == tankId)) {
    pongPending = true;
  } else if (h->tank == tankId) {
    if (h->type == RV_DRIVE && length >= sizeof(RvDrive)) {
      const RvDrive *d = (const RvDrive *)recv_buff;
      lastCmdStamp = millis();
      drive(d->fwd, d->turn);
      setFire(d->fire);
    } else if (h->type == RV_CAM && length >= sizeof(RvCam)) {
      memcpy(&camNew, recv_buff, sizeof(RvCam));
      camPending = true;
    }
  }
}

void applyCam() {
  RvCam c = camNew;
  camPending = false;
  sensor_t *s = Camera.sensor;
  if (!s) return;
  bool first = !camAppliedValid;
  if (c.size < 3 && (first || c.size != camApplied.size)) s->set_framesize(s, frameSizes[c.size]);
  if (first || c.brightness != camApplied.brightness) s->set_brightness(s, constrain(c.brightness, -2, 2));
  if (first || c.contrast != camApplied.contrast) s->set_contrast(s, constrain(c.contrast, -2, 2));
  if (first || c.saturation != camApplied.saturation) s->set_saturation(s, constrain(c.saturation, -2, 2));
  if (first || c.hmirror != camApplied.hmirror) s->set_hmirror(s, c.hmirror);
  if (first || c.vflip != camApplied.vflip) s->set_vflip(s, c.vflip);
  jpgQuality = constrain(c.quality, 5, 50);
  camApplied = c;
  camAppliedValid = true;
}

void sendPong() {
  RvHdr h = {RV_MAGIC, RV_PONG, tankId};
  pongPending = false;
  radio.sendData((uint8_t *)&h, sizeof(h));
}

void processFrame() {
  if (Camera.get()) {
    uint8_t *out_jpg = NULL;
    size_t out_jpg_len = 0;
    frame2jpg(Camera.fb, jpgQuality, &out_jpg, &out_jpg_len);
    radio.sendData(out_jpg, out_jpg_len);
    free(out_jpg);
    Camera.free();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  tankId = readTankId();
  Serial.printf("Tank id / channel: %u\r\n", tankId);

  if (FIRE_PIN >= 0) {
    pinMode(FIRE_PIN, OUTPUT);
    setFire(false);
  }

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servoLeft.setPeriodHertz(50);
  servoRight.setPeriodHertz(50);

  if (Camera.begin()) Serial.println("Camera Init Success");
  else Serial.println("Camera Init Failed");

  recv_buff = static_cast<uint8_t *>(ps_malloc(100));
  radio.setRecvBuffer(recv_buff);
  radio.setRecvCallback(onDataReady);
  if (radio.init(244)) Serial.println("ESPNow Init Success");
  esp_wifi_set_channel(tankId, WIFI_SECOND_CHAN_NONE);
}

void loop() {
  processFrame();
  if (pongPending) sendPong();
  if (camPending) applyCam();
  // failsafe: stop if the controller is gone
  if (millis() - lastCmdStamp > CMD_TIMEOUT_MS) {
    drive(0, 0);
    setFire(false);
  }
}
