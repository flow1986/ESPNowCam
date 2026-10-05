/**************************************************
 * CYD (ESP32-2432S028R) Rover Controller
 * Receives the ESP-NOW camera stream and sends drive commands read
 * from a PCF8574 I2C GPIO expander (CYD-GB buttons) to the rover.
 * This file is part ESPNowCam project:
 * https://github.com/hpsaturn/ESPNowCam
 *
 * Button mapping (PCF bit => function):
 *   0 UP    forward        4 A      boost speed
 *   1 DOWN  backward       5 B      slow speed
 *   2 LEFT  turn left      6 START  (unused)
 *   3 RIGHT turn right     7 SELECT (unused)
**************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>
#include <ESPNowCam.h>
#include <common/comm.pb.h>

#define PCF_ADDR 0x20
#define PCF_SDA 16
#define PCF_SCL 17

#define SPEED_NORMAL 60
#define SPEED_BOOST 100
#define SPEED_SLOW 30

// 1: camera image rotated 90 degrees to fill the portrait screen, 0: landscape
#ifndef VIEW_ROTATE
#define VIEW_ROTATE 1
#endif

#define IMG_W 320
#define IMG_H 240

ESPNowCam radio;
TFT_eSPI tft;

static uint8_t fb[40000];   // ESPNowCam receive buffer
static uint8_t jpg[24000];  // frame copy for decoding outside of the radio callback
static volatile uint32_t jpg_len = 0;
static volatile bool frame_ready = false;
static uint32_t lastFrameStamp = 0;
static bool noSignalDrawn = false;

// frame callback runs in the WiFi task, so only copy the frame here
void onFrameReady(uint32_t length) {
  if (frame_ready || length > sizeof(jpg)) return;
  memcpy(jpg, fb, length);
  jpg_len = length;
  frame_ready = true;
}

bool tftOutput(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
#if VIEW_ROTATE
  static uint16_t blk[16 * 16];
  if (w > 16 || h > 16) return true;
  for (int j = 0; j < h; j++)
    for (int i = 0; i < w; i++) blk[i * h + (h - 1 - j)] = bitmap[j * w + i];
  tft.pushImage(IMG_H - y - h, x, h, w, blk);
#else
  tft.pushImage(x, y, w, h, bitmap);
#endif
  return true;
}

uint8_t readButtons() {
  Wire.requestFrom((uint8_t)PCF_ADDR, (uint8_t)1);
  if (Wire.available() < 1) return 0;
  return ~Wire.read();  // pressed = low
}

void sendDrive(int8_t fwd, int8_t turn) {
  static uint8_t buf[64];
  JoystickMessage jm = JoystickMessage_init_zero;
  jm.ay = 100 + fwd;   // >100 forward
  jm.az = 100 + turn;  // >100 turn right
  jm.ck = 0x01;
  pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
  if (!pb_encode(&stream, JoystickMessage_fields, &jm)) return;
  radio.sendData(buf, stream.bytes_written);
}

void controlTask(void *arg) {
  int8_t lastFwd = 0, lastTurn = 0;
  uint32_t lastSend = 0;
  for (;;) {
    uint8_t b = readButtons();
    int speed = (b & 0x10) ? SPEED_BOOST : (b & 0x20) ? SPEED_SLOW : SPEED_NORMAL;
    int8_t fwd = ((b & 0x01) ? speed : 0) - ((b & 0x02) ? speed : 0);
    int8_t turn = ((b & 0x08) ? speed : 0) - ((b & 0x04) ? speed : 0);

    // resend as keep-alive, the rover stops if commands are missing
    if (fwd != lastFwd || turn != lastTurn || millis() - lastSend > 100) {
      sendDrive(fwd, turn);
      lastFwd = fwd;
      lastTurn = turn;
      lastSend = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void drawNoSignal() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("NO CAMERA SIGNAL", tft.width() / 2, tft.height() / 2, 2);
}

void setup() {
  Serial.begin(115200);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  tft.init();
  tft.invertDisplay(true);
  tft.setSwapBytes(true);
  tft.setRotation(VIEW_ROTATE ? 2 : 1);
  tft.fillScreen(TFT_BLACK);

  TJpgDec.setJpgScale(1);
  TJpgDec.setCallback(tftOutput);

  Wire.begin(PCF_SDA, PCF_SCL);
  Wire.setClock(100000);
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(0xFF);  // all pins as inputs
  Wire.endTransmission();

  radio.setRecvBuffer(fb);
  radio.setRecvCallback(onFrameReady);
  if (radio.init()) Serial.println("ESPNow Init Success");

  xTaskCreatePinnedToCore(controlTask, "ctrl", 4096, NULL, 2, NULL, 0);
  drawNoSignal();
  noSignalDrawn = true;
  lastFrameStamp = millis();
}

void loop() {
  if (frame_ready) {
    TJpgDec.drawJpg(0, 0, jpg, jpg_len);
    frame_ready = false;
    lastFrameStamp = millis();
    noSignalDrawn = false;
  } else if (!noSignalDrawn && millis() - lastFrameStamp > 2000) {
    drawNoSignal();
    noSignalDrawn = true;
  }
  delay(1);
}
