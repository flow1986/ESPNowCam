/**************************************************
 * CYD (ESP32-2432S028R) Rover Controller
 * Receives the ESP-NOW camera stream of one tank and sends drive
 * commands read from a PCF8574 I2C GPIO expander (CYD-GB buttons).
 * This file is part ESPNowCam project:
 * https://github.com/hpsaturn/ESPNowCam
 *
 * Drive:  A forward, B backward, LEFT/RIGHT turn
 *         UP/DOWN speed +10% / -10%, SELECT speed back to 50%
 *         START fire (held), START+SELECT opens the config menu
 * Menu:   UP/DOWN select, LEFT/RIGHT change, A scan/close, B close
**************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <esp_wifi.h>
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>
#include <ESPNowCam.h>
#include <common/rover_proto.h>

#define PCF_ADDR 0x20
#define PCF_SDA 16
#define PCF_SCL 17

// PCF8574 bits (pressed = low)
#define BTN_UP 0x01
#define BTN_DOWN 0x02
#define BTN_LEFT 0x04
#define BTN_RIGHT 0x08
#define BTN_A 0x10
#define BTN_B 0x20
#define BTN_START 0x40
#define BTN_SELECT 0x80

#define SPEED_DEFAULT 50

// 1: camera image rotated 90 degrees to fill the portrait screen, 0: landscape
#ifndef VIEW_ROTATE
#define VIEW_ROTATE 1
#endif

// the menu is drawn below this line, above it the live image stays visible (portrait only)
#define MENU_TOP (VIEW_ROTATE ? 96 : 0)
#define ROW_H 18

struct Cfg {
  uint8_t tank = 1;
  uint8_t size = 2;
  uint8_t quality = 15;
  int8_t brightness = 0;
  int8_t contrast = 0;
  int8_t saturation = 0;
  uint8_t hmirror = 0;
  uint8_t vflip = 0;
  uint8_t crosshair = 1;
};

ESPNowCam radio;
TFT_eSPI tft;
Preferences prefs;
static Cfg cfg;

static uint8_t fb[40000];   // ESPNowCam receive buffer
static uint8_t jpg[32000];  // frame copy for decoding outside of the radio callback
static volatile uint32_t jpg_len = 0;
static volatile bool frame_ready = false;
static uint32_t lastFrameStamp = 0;
static bool noSignalDrawn = false;
static int imgW = 0, imgH = 0, offX = 0, offY = 0;

static SemaphoreHandle_t radioMutex;
static volatile uint8_t btnState = 0;
static volatile uint8_t speedPct = SPEED_DEFAULT;
static volatile bool menuOpen = false;
static volatile bool menuDirty = false;
static volatile bool scanning = false;
static volatile bool camDirty = true;
static volatile uint16_t foundMask = 0;
static volatile uint32_t lastPong = 0;
static char scanMsg[32] = "";
static int menuSel = 0;

#define MENU_ITEMS 11
static const char *labels[MENU_ITEMS] = {"Tank",       "Scan tanks", "Resolution", "JPEG quality",
                                         "Brightness", "Contrast",   "Saturation", "Mirror",
                                         "Flip",       "Crosshair",  "Close"};
static const char *sizeNames[3] = {"160x120", "240x176", "320x240"};

bool tankOnline() { return millis() - lastPong < 2000 || millis() - lastFrameStamp < 2000; }

void loadCfg() {
  prefs.begin("rover", false);
  Cfg tmp;
  if (prefs.getBytes("cfg", &tmp, sizeof(tmp)) == sizeof(tmp)) cfg = tmp;
  cfg.tank = constrain(cfg.tank, 1, RV_MAX_TANKS);
}

void saveCfg() { prefs.putBytes("cfg", &cfg, sizeof(cfg)); }

void sendPacket(const void *p, size_t n) {
  xSemaphoreTake(radioMutex, portMAX_DELAY);
  radio.sendData((uint8_t *)p, n);
  xSemaphoreGive(radioMutex);
}

void applyChannel(uint8_t ch) {
  xSemaphoreTake(radioMutex, portMAX_DELAY);
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  xSemaphoreGive(radioMutex);
}

// runs in the WiFi task, so only copy the frame here
void onRecv(uint32_t length) {
  if (length >= sizeof(RvHdr) && fb[0] == RV_MAGIC) {
    const RvHdr *h = (const RvHdr *)fb;
    if (h->type == RV_PONG && h->tank >= 1 && h->tank <= RV_MAX_TANKS) {
      foundMask |= 1 << h->tank;
      if (h->tank == cfg.tank) lastPong = millis();
    }
    return;
  }
  if (frame_ready || length > sizeof(jpg)) return;
  memcpy(jpg, fb, length);
  jpg_len = length;
  frame_ready = true;
}

bool tftOutput(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
#if VIEW_ROTATE
  static uint16_t blk[16 * 16];
  if (w > 16 || h > 16) return true;
  int dx = offX + imgH - y - h, dy = offY + x;
  if (menuOpen && dy + w > MENU_TOP) return true;
  for (int j = 0; j < h; j++)
    for (int i = 0; i < w; i++) blk[i * h + (h - 1 - j)] = bitmap[j * w + i];
  tft.pushImage(dx, dy, h, w, blk);
#else
  if (menuOpen) return true;
  tft.pushImage(offX + x, offY + y, w, h, bitmap);
#endif
  return true;
}

uint8_t readButtons() {
  Wire.requestFrom((uint8_t)PCF_ADDR, (uint8_t)1);
  if (Wire.available() < 1) return 0;
  return ~Wire.read();
}

void controlTask(void *arg) {
  uint8_t prev = 0;
  bool selArmed = false, suppress = false, comboPrev = false;
  int8_t lastFwd = 0, lastTurn = 0;
  uint8_t lastFire = 0;
  uint32_t lastSend = 0, lastCam = 0, lastPing = 0;

  for (;;) {
    uint8_t b = readButtons();
    btnState = b;
    uint8_t pressed = b & ~prev;
    uint8_t released = prev & ~b;
    prev = b;
    uint32_t now = millis();

    bool combo = (b & BTN_START) && (b & BTN_SELECT);
    if (combo && !comboPrev) {
      menuOpen = !menuOpen;
      menuDirty = true;
    }
    comboPrev = combo;

    // select alone resets the speed, but not as part of the menu combo
    if (pressed & BTN_SELECT) selArmed = true;
    if (combo) selArmed = false;
    if ((released & BTN_SELECT) && selArmed && !menuOpen && !suppress) speedPct = SPEED_DEFAULT;
    if (released & BTN_SELECT) selArmed = false;

    // after closing the menu ignore the keys until they are released
    if (menuOpen) suppress = true;
    else if (suppress && !(b & 0x3F)) suppress = false;

    int8_t fwd = 0, turn = 0;
    uint8_t fire = 0;
    if (!menuOpen && !suppress) {
      if ((pressed & BTN_UP) && speedPct < 100) speedPct = speedPct + 10;
      if ((pressed & BTN_DOWN) && speedPct > 0) speedPct = speedPct - 10;
      int s = speedPct;
      fwd = ((b & BTN_A) ? s : 0) - ((b & BTN_B) ? s : 0);
      turn = ((b & BTN_RIGHT) ? s : 0) - ((b & BTN_LEFT) ? s : 0);
      fire = (b & BTN_START) && !(b & BTN_SELECT);
    }

    if (!scanning) {
      // keep-alive: the tank stops if commands are missing
      if (fwd != lastFwd || turn != lastTurn || fire != lastFire || now - lastSend > 100) {
        RvDrive d = {{RV_MAGIC, RV_DRIVE, cfg.tank}, fwd, turn, fire};
        sendPacket(&d, sizeof(d));
        lastFwd = fwd;
        lastTurn = turn;
        lastFire = fire;
        lastSend = now;
      }
      if (camDirty || now - lastCam > 1000) {
        camDirty = false;
        RvCam c = {{RV_MAGIC, RV_CAM, cfg.tank}, cfg.size,       cfg.quality, cfg.brightness,
                   cfg.contrast,                 cfg.saturation, cfg.hmirror, cfg.vflip};
        sendPacket(&c, sizeof(c));
        lastCam = now;
      }
      if (now - lastPing > 500) {
        RvHdr p = {RV_MAGIC, RV_PING, cfg.tank};
        sendPacket(&p, sizeof(p));
        lastPing = now;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void menuValue(int i, char *out, size_t n) {
  switch (i) {
    case 0: snprintf(out, n, "< %u > %s", cfg.tank, tankOnline() ? "ONLINE" : "offline"); break;
    case 1: snprintf(out, n, "A = scan"); break;
    case 2: snprintf(out, n, "< %s >", sizeNames[cfg.size]); break;
    case 3: snprintf(out, n, "< %u >", cfg.quality); break;
    case 4: snprintf(out, n, "< %d >", cfg.brightness); break;
    case 5: snprintf(out, n, "< %d >", cfg.contrast); break;
    case 6: snprintf(out, n, "< %d >", cfg.saturation); break;
    case 7: snprintf(out, n, "%s", cfg.hmirror ? "on" : "off"); break;
    case 8: snprintf(out, n, "%s", cfg.vflip ? "on" : "off"); break;
    case 9: snprintf(out, n, "%s", cfg.crosshair ? "on" : "off"); break;
    default: snprintf(out, n, "A / B"); break;
  }
}

void drawMenu() {
  char val[24];
  int y = MENU_TOP + 4;
  for (int i = 0; i < MENU_ITEMS; i++, y += ROW_H) {
    uint16_t bg = (i == menuSel) ? TFT_NAVY : TFT_BLACK;
    tft.fillRect(0, y, tft.width(), ROW_H, bg);
    tft.setTextColor(TFT_WHITE, bg);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(labels[i], 4, y + 1, 2);
    menuValue(i, val, sizeof(val));
    tft.setTextDatum(TR_DATUM);
    tft.drawString(val, tft.width() - 4, y + 1, 2);
  }
  tft.fillRect(0, y, tft.width(), ROW_H, TFT_BLACK);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(scanMsg, 4, y + 1, 2);
}

void scanTanks() {
  scanning = true;
  foundMask = 0;
  snprintf(scanMsg, sizeof(scanMsg), "Scanning...");
  drawMenu();
  RvHdr ping = {RV_MAGIC, RV_PING, 0};
  for (uint8_t ch = 1; ch <= RV_MAX_TANKS; ch++) {
    applyChannel(ch);
    for (int k = 0; k < 2; k++) {
      sendPacket(&ping, sizeof(ping));
      delay(70);
    }
    delay(60);
  }
  uint16_t mask = foundMask;
  int n = snprintf(scanMsg, sizeof(scanMsg), "Found:");
  for (int t = 1; t <= RV_MAX_TANKS; t++)
    if (mask & (1 << t)) n += snprintf(scanMsg + n, sizeof(scanMsg) - n, " %d", t);
  if (!mask) snprintf(scanMsg, sizeof(scanMsg), "No tank found");
  // jump to the first tank found unless the current one is among them
  else if (!(mask & (1 << cfg.tank)))
    for (int t = 1; t <= RV_MAX_TANKS; t++)
      if (mask & (1 << t)) {
        cfg.tank = t;
        break;
      }
  applyChannel(cfg.tank);
  lastPong = 0;
  camDirty = true;
  scanning = false;
}

void changeValue(int item, int d) {
  switch (item) {
    case 0:
      cfg.tank = (cfg.tank - 1 + d + RV_MAX_TANKS) % RV_MAX_TANKS + 1;
      applyChannel(cfg.tank);
      lastPong = 0;
      break;
    case 2: cfg.size = constrain(cfg.size + d, 0, 2); break;
    case 3: cfg.quality = constrain(cfg.quality + 5 * d, 5, 50); break;
    case 4: cfg.brightness = constrain(cfg.brightness + d, -2, 2); break;
    case 5: cfg.contrast = constrain(cfg.contrast + d, -2, 2); break;
    case 6: cfg.saturation = constrain(cfg.saturation + d, -2, 2); break;
    case 7: cfg.hmirror ^= 1; break;
    case 8: cfg.vflip ^= 1; break;
    case 9: cfg.crosshair ^= 1; break;
  }
  if (item >= 0 && item <= 8) camDirty = true;
}

void menuInput(uint8_t b) {
  static uint8_t prev = 0;
  static uint32_t holdStart = 0, lastRepeat = 0;
  const uint8_t dirs = BTN_UP | BTN_DOWN | BTN_LEFT | BTN_RIGHT;
  uint32_t now = millis();
  uint8_t ev = b & ~prev;
  if (ev & dirs) {
    holdStart = lastRepeat = now;
  } else if ((b & dirs) && now - holdStart > 400 && now - lastRepeat > 120) {
    ev |= b & dirs;
    lastRepeat = now;
  }
  prev = b;
  if (!ev) return;

  if (ev & BTN_UP) menuSel = (menuSel + MENU_ITEMS - 1) % MENU_ITEMS;
  if (ev & BTN_DOWN) menuSel = (menuSel + 1) % MENU_ITEMS;
  if (ev & BTN_LEFT) changeValue(menuSel, -1);
  if (ev & BTN_RIGHT) changeValue(menuSel, 1);
  if ((ev & BTN_A) && menuSel == 1) scanTanks();
  if (((ev & BTN_A) && menuSel == MENU_ITEMS - 1) || (ev & BTN_B)) menuOpen = false;
  menuDirty = true;
}

void drawCrosshair() {
  int cx = tft.width() / 2, cy = tft.height() / 2;
  tft.drawCircle(cx, cy, 14, TFT_RED);
  tft.drawFastHLine(cx - 22, cy, 14, TFT_RED);
  tft.drawFastHLine(cx + 9, cy, 14, TFT_RED);
  tft.drawFastVLine(cx, cy - 22, 14, TFT_RED);
  tft.drawFastVLine(cx, cy + 9, 14, TFT_RED);
}

void drawHud() {
  char s[24];
  snprintf(s, sizeof(s), "T%u  SPD %u%%", cfg.tank, speedPct);
  tft.fillRect(0, 0, 76, 11, TFT_BLACK);
  tft.setTextColor(tankOnline() ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(s, 2, 2, 1);
}

void drawNoSignal() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("NO CAMERA SIGNAL", tft.width() / 2, tft.height() / 2, 2);
}

void drawFrame() {
  uint16_t w = 0, h = 0;
  TJpgDec.getJpgSize(&w, &h, jpg, jpg_len);
  if (w == 0 || h == 0) return;
  if (w != imgW || h != imgH) {
    imgW = w;
    imgH = h;
    tft.fillScreen(TFT_BLACK);
    if (menuOpen) menuDirty = true;
  }
#if VIEW_ROTATE
  offX = (tft.width() - imgH) / 2;
  offY = (tft.height() - imgW) / 2;
#else
  offX = (tft.width() - imgW) / 2;
  offY = (tft.height() - imgH) / 2;
#endif
  TJpgDec.drawJpg(0, 0, jpg, jpg_len);
  if (!menuOpen) {
    if (cfg.crosshair) drawCrosshair();
    drawHud();
  }
}

void setup() {
  Serial.begin(115200);
  loadCfg();
  radioMutex = xSemaphoreCreateMutex();

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
  radio.setRecvCallback(onRecv);
  if (radio.init()) Serial.println("ESPNow Init Success");
  applyChannel(cfg.tank);

  xTaskCreatePinnedToCore(controlTask, "ctrl", 4096, NULL, 2, NULL, 0);
  drawNoSignal();
  noSignalDrawn = true;
  lastFrameStamp = millis();
}

void loop() {
  static bool menuShown = false;
  if (menuOpen != menuShown) {
    menuShown = menuOpen;
    tft.fillScreen(TFT_BLACK);
    if (menuOpen) menuDirty = true;
    else saveCfg();
  }

  if (menuOpen) {
    menuInput(btnState);
    if (menuDirty) {
      menuDirty = false;
      drawMenu();
    }
  }

  if (frame_ready) {
    drawFrame();
    frame_ready = false;
    lastFrameStamp = millis();
    noSignalDrawn = false;
  } else if (!menuOpen && !noSignalDrawn && millis() - lastFrameStamp > 2000) {
    drawNoSignal();
    noSignalDrawn = true;
  }
  delay(1);
}
