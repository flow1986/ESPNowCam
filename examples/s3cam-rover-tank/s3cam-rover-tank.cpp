/**************************************************
 * ESP32-S3 CAM Rover (Freenove pinout)
 * Streams the camera via ESP-NOW and drives two continuous rotation
 * servos (tank) from the commands of the CYD rover controller.
 * This file is part ESPNowCam project:
 * https://github.com/hpsaturn/ESPNowCam
**************************************************/

#include <Arduino.h>
#include <ESPNowCam.h>
#include <common/comm.pb.h>
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
#define CMD_TIMEOUT_MS 500

CamFreenove Camera;
ESPNowCam radio;
Servo servoLeft;
Servo servoRight;

static uint8_t *recv_buff;
static volatile uint32_t lastCmdStamp = 0;

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

void onDataReady(uint32_t length) {
  JoystickMessage jm = JoystickMessage_init_zero;
  pb_istream_t stream = pb_istream_from_buffer(recv_buff, length);
  if (!pb_decode(&stream, JoystickMessage_fields, &jm) || jm.ck != 0x01) return;
  lastCmdStamp = millis();
  drive(jm.ay - 100, jm.az - 100);
}

void processFrame() {
  if (Camera.get()) {
    uint8_t *out_jpg = NULL;
    size_t out_jpg_len = 0;
    frame2jpg(Camera.fb, 12, &out_jpg, &out_jpg_len);
    radio.sendData(out_jpg, out_jpg_len);
    free(out_jpg);
    Camera.free();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

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
}

void loop() {
  processFrame();
  // failsafe: stop if the controller is gone
  if (millis() - lastCmdStamp > CMD_TIMEOUT_MS) drive(0, 0);
}
