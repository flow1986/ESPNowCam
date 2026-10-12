/**************************************************
 * ESP32-S3 CAM Rover (Freenove pinout)
 * Streams the camera and drives two continuous rotation servos (tank)
 * plus a gripper (grip + lift servo) from a remote: the CYD rover
 * controller (ESP-NOW or WiFi) or the web UI of the tank (WiFi).
 *
 * DIP switch (3 pins): 1..7 = ESP-NOW mode, tank id = ESP-NOW channel.
 *                      0 (all off, also no switch fitted) = WiFi mode.
 * WiFi mode: joins the saved WLAN, otherwise opens the AP "RoverTank-Setup"
 * with the config page http://192.168.4.1/wifi. Web UI on port 80,
 * WebSocket (same packets as ESP-NOW) on port 81, mDNS name rovertank<id>.
 * Camera and servo settings are stored on the tank (NVS).
 *
 * This file is part ESPNowCam project:
 * https://github.com/hpsaturn/ESPNowCam
**************************************************/

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WebSocketsServer.h>
#include <esp_wifi.h>
#include <ESPNowCam.h>
#include <common/rover_proto.h>
#include <drivers/CamFreenove.h>
#include <ESP32Servo.h>
#include "web_ui.h"

#ifndef SERVO_L_PIN
#define SERVO_L_PIN 47
#endif
#ifndef SERVO_R_PIN
#define SERVO_R_PIN 21
#endif
// pulse width offset at 100% speed (the neutral pulse is calibrated on the remote)
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
// DIP switch to GND, bit0..bit2 (ON = 1). 1..7 = ESP-NOW id / channel, 0 = WiFi mode
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
#ifndef AP_SSID
#define AP_SSID "RoverTank-Setup"
#endif
#ifndef AP_PASSWORD
#define AP_PASSWORD "rovertank"
#endif
#define CMD_TIMEOUT_MS 500
#define CFG_SAVE_DELAY_MS 2000
// max WebSocket message payload for video (the library rejects frames above 15 KB)
#define WS_CHUNK 12000

CamFreenove Camera;
ESPNowCam radio;
Servo servoLeft;
Servo servoRight;
Servo servoGrip;
Servo servoLift;
Preferences tp;
WebServer web(80);
WebSocketsServer wsServer(81);

static bool wifiMode = false;
static bool apMode = false;
static uint8_t *recv_buff;
static uint8_t tankId = 1;
static volatile uint32_t lastCmdStamp = 0;
static volatile bool pongPending = false;
static volatile bool cfgReplyPending = false;
static volatile bool camPending = false;
static volatile int wsClients = 0;
static camera_fb_t *volatile wifiFrame = nullptr;

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
  return v;
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
 * Packets (ESP-NOW and WebSocket)
************************************/
static bool differs(const void *a, const void *b, size_t skipEnd) {
  return memcmp((const uint8_t *)a + sizeof(RvHdr), (const uint8_t *)b + sizeof(RvHdr), sizeof(RvCam) - sizeof(RvHdr) - skipEnd) != 0;
}

void onCamPacket(const uint8_t *d) {
  RvCam c;
  memcpy(&c, d, sizeof(c));
  if (differs(&c, &camCfg, 0)) {
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
  bool changed = memcmp((const uint8_t *)&s + sizeof(RvHdr), (const uint8_t *)&sv + sizeof(RvHdr),
                        sizeof(RvServo) - sizeof(RvHdr) - 1) != 0;
  sv = s;
  sv.h = {RV_MAGIC, RV_SERVO, tankId};
  if (changed) {
    cfgDirty = true;
    cfgDirtyStamp = millis();
  }
  // the remote sends zero drive commands while calibrating, so this can't interrupt driving
  if (sv.preview == 1 || oldPreview == 1) drive(0, 0);
  updateGripper();
}

// runs in the WiFi task (ESP-NOW) or in the net task (WebSocket): no sends and no sensor access here
void handlePacket(const uint8_t *buf, uint32_t length) {
  if (length < sizeof(RvHdr)) return;
  const RvHdr *h = (const RvHdr *)buf;
  if (h->magic != RV_MAGIC) return;
  bool mine = (h->tank == tankId);
  bool any = (h->tank == 0 || mine);

  if (h->type == RV_PING && any) {
    pongPending = true;
  } else if (h->type == RV_GETCFG && any) {
    cfgReplyPending = true;
  } else if (mine) {
    if (h->type == RV_DRIVE && length >= sizeof(RvDrive)) {
      const RvDrive *d = (const RvDrive *)buf;
      lastCmdStamp = millis();
      drive(d->fwd, d->turn);
      setFire(d->fire);
      gripPct = constrain(d->grip, 0, 100);
      liftPct = constrain(d->lift, 0, 100);
      gripperActive = true;
      updateGripper();
    } else if (h->type == RV_CAM && length >= sizeof(RvCam)) {
      onCamPacket(buf);
    } else if (h->type == RV_SERVO && length >= sizeof(RvServo)) {
      onServoPacket(buf);
    }
  }
}

void onDataReady(uint32_t length) { handlePacket(recv_buff, length); }

void sendOut(const void *p, size_t n) {
  if (wifiMode) wsServer.broadcastBIN((uint8_t *)p, n);
  else radio.sendData((uint8_t *)p, n);
}

// replies are sent from the loop that owns the radio / the WebSocket server
void serviceReplies() {
  if (pongPending) {
    pongPending = false;
    RvPong r = {{RV_MAGIC, RV_PONG, tankId}, (uint8_t)gripPct, (uint8_t)liftPct};
    sendOut(&r, sizeof(r));
  }
  if (cfgReplyPending) {
    cfgReplyPending = false;
    RvCam c = camCfg;
    sendOut(&c, sizeof(c));
    RvServo s = sv;
    s.preview = 0;
    sendOut(&s, sizeof(s));
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
  if (wifiMode) {
    // hand the frame to the net task, drop it if that is still busy or nobody is connected
    if (wsClients > 0 && wifiFrame == nullptr) {
      wifiFrame = Camera.fb;
      Camera.fb = nullptr;
    } else {
      Camera.free();
      if (wsClients == 0) delay(30);
    }
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

/***********************************
 * WiFi mode: web UI, config page, WebSocket
************************************/
static String htmlEsc(const String &s) {
  String o;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '&') o += "&amp;";
    else if (c == '<') o += "&lt;";
    else if (c == '>') o += "&gt;";
    else if (c == '"') o += "&quot;";
    else if (c == '\'') o += "&#39;";
    else o += c;
  }
  return o;
}

void handleWifiPage() {
  String options;
  if (apMode) {
    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; i++) options += "<option value=\"" + htmlEsc(WiFi.SSID(i)) + "\">";
  }
  String page =
      F("<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
        "<title>Rover WiFi</title><body style='font-family:sans-serif;max-width:460px;margin:auto;padding:10px'>"
        "<h2>Rover tank WiFi</h2><form method=post action=/wifi>"
        "WLAN name (SSID)<br><input name=ssid list=nets style='width:100%' value=\"");
  page += htmlEsc(tp.getString("ssid", ""));
  page += F("\"><datalist id=nets>");
  page += options;
  page += F("</datalist><br><br>Password<br><input name=pass type=password style='width:100%' placeholder='leave empty to keep'><br><br>"
            "Tank id (1-7, must be unique in the WLAN)<br><input name=id type=number min=1 max=7 value=");
  page += String(tankId);
  page += F("><br><br><button>Save and restart</button></form><p>Status: ");
  if (WiFi.status() == WL_CONNECTED) page += "connected to " + htmlEsc(WiFi.SSID()) + ", IP " + WiFi.localIP().toString();
  else page += "setup access point";
  page += F("</p><p><a href=/>Back to the control page</a></p>");
  web.send(200, "text/html", page);
}

void handleWifiSave() {
  String ssid = web.arg("ssid");
  String pass = web.arg("pass");
  int id = constrain(web.arg("id").toInt(), 1, RV_MAX_TANKS);
  tp.putString("ssid", ssid);
  if (pass.length() > 0 || ssid.length() == 0) tp.putString("pass", pass);
  tp.putUChar("id", id);
  web.send(200, "text/html", F("<meta charset=utf-8><body style='font-family:sans-serif'>Saved, the tank restarts. "
                                "Reconnect to your WLAN and open <b>http://rovertank&lt;id&gt;.local</b> or the IP shown in the serial monitor."));
  delay(800);
  ESP.restart();
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      wsClients++;
      pongPending = true;
      cfgReplyPending = true;
      break;
    case WStype_DISCONNECTED:
      if (wsClients > 0) wsClients--;
      break;
    case WStype_BIN: handlePacket(payload, length); break;
    default: break;
  }
}

void netTask(void *arg) {
  static uint8_t chunkBuf[2 + WS_CHUNK];
  uint32_t winStart = millis(), frames = 0, bytes = 0;
  for (;;) {
    wsServer.loop();
    web.handleClient();
    serviceReplies();
    camera_fb_t *f = wifiFrame;
    if (f) {
      if (wsClients > 0) {
        size_t off = 0;
        while (off < f->len) {
          size_t n = min((size_t)WS_CHUNK, f->len - off);
          chunkBuf[0] = 0xFE;
          chunkBuf[1] = (off == 0 ? 1 : 0) | (off + n >= f->len ? 2 : 0);
          memcpy(chunkBuf + 2, f->buf + off, n);
          wsServer.broadcastBIN(chunkBuf, n + 2);
          off += n;
        }
        frames++;
        bytes += f->len;
      }
      esp_camera_fb_return(f);
      wifiFrame = nullptr;
    }
    if (millis() - winStart >= 2000) {
      Serial.printf("WS %.1f fps, %.1f KB/frame, %d client(s)\r\n", frames * 1000.0f / (millis() - winStart),
                    frames ? bytes / 1024.0f / frames : 0.0f, (int)wsClients);
      winStart = millis();
      frames = bytes = 0;
    }
    vTaskDelay(1);
  }
}

void startWifiMode() {
  String ssid = tp.getString("ssid", "");
  String pass = tp.getString("pass", "");
  WiFi.persistent(false);
  bool connected = false;
  if (ssid.length() > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(ssid.c_str(), pass.c_str());
    Serial.printf("Joining WLAN \"%s\" ", ssid.c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
      delay(250);
      Serial.print(".");
    }
    Serial.println();
    connected = WiFi.status() == WL_CONNECTED;
  }
  if (connected) {
    Serial.printf("WLAN connected, IP %s\r\n", WiFi.localIP().toString().c_str());
  } else {
    apMode = true;
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.printf("Setup access point \"%s\" (password \"%s\"), config page http://%s/wifi\r\n", AP_SSID, AP_PASSWORD,
                  WiFi.softAPIP().toString().c_str());
  }

  char host[24];
  snprintf(host, sizeof(host), "rovertank%u", tankId);
  if (MDNS.begin(host)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("mDNS: http://%s.local\r\n", host);
  }

  web.on("/", HTTP_GET, []() { web.send_P(200, "text/html", WEB_UI); });
  web.on("/wifi", HTTP_GET, handleWifiPage);
  web.on("/wifi", HTTP_POST, handleWifiSave);
  web.onNotFound([]() {
    web.sendHeader("Location", "/");
    web.send(302, "text/plain", "");
  });
  web.begin();
  wsServer.begin();
  wsServer.onEvent(onWsEvent);
  xTaskCreatePinnedToCore(netTask, "net", 8192, NULL, 2, NULL, 0);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  tp.begin("tank", false);
  uint8_t dip = readDip();
  wifiMode = (dip == 0);
  tankId = wifiMode ? constrain(tp.getUChar("id", 1), 1, RV_MAX_TANKS) : dip;
  Serial.printf("Mode: %s, tank id %u\r\n", wifiMode ? "WiFi" : "ESP-NOW", tankId);
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

  if (wifiMode) {
    startWifiMode();
  } else {
    recv_buff = static_cast<uint8_t *>(ps_malloc(100));
    radio.setRecvBuffer(recv_buff);
    radio.setRecvCallback(onDataReady);
    if (radio.init(244)) Serial.println("ESPNow Init Success");
    esp_wifi_set_channel(tankId, WIFI_SECOND_CHAN_NONE);
  }
}

void loop() {
  processFrame();
  if (!wifiMode) serviceReplies();
  if (camPending) applyCam();
  saveCfgIfDue();
  // failsafe: stop driving if the remote is gone, the gripper keeps its position
  if (millis() - lastCmdStamp > CMD_TIMEOUT_MS) {
    drive(0, 0);
    setFire(false);
  }
}
