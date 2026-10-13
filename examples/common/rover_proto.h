// Packets shared by the CYD rover controller and the S3 CAM rover tank.
// Transport: ESPNowCam sendData() (ESP-NOW).
// All fields are little endian and packed. Video frames are raw JPEG (start with 0xFF),
// control packets start with RV_MAGIC.
#pragma once
#include <stdint.h>

#define RV_MAGIC 0xA5
#define RV_MAX_TANKS 7  // tank id 1..7 (id == ESP-NOW channel == DIP switch value, no switch fitted = 1)

enum RvType : uint8_t {
  RV_DRIVE = 1,   // controller -> tank
  RV_CAM = 2,     // camera settings: controller -> tank (stored on the tank), tank -> controller (reply to RV_GETCFG)
  RV_PING = 3,    // controller -> tank, tank 0 = any tank on this channel
  RV_PONG = 4,    // tank -> controller, RvPong
  RV_SERVO = 5,   // servo calibration: same direction rules as RV_CAM
  RV_GETCFG = 6   // controller -> tank, the tank answers with RV_CAM and RV_SERVO
};

struct __attribute__((packed)) RvHdr {
  uint8_t magic;
  uint8_t type;
  uint8_t tank;
};

struct __attribute__((packed)) RvDrive {
  RvHdr h;
  int8_t fwd;   // -100..100, >0 forward
  int8_t turn;  // -100..100, >0 right
  uint8_t fire;
  uint8_t grip;  // gripper position 0..100 (0 = min pulse, 100 = max pulse)
  uint8_t lift;  // gripper lift position 0..100
};

struct __attribute__((packed)) RvPong {
  RvHdr h;
  uint8_t grip;  // current gripper / lift position, so a controller can continue where the last one stopped
  uint8_t lift;
};

struct __attribute__((packed)) RvCam {
  RvHdr h;
  uint8_t size;  // 0=160x120 1=240x176 2=320x240
  uint8_t quality;
  int8_t brightness;  // -2..2
  int8_t contrast;
  int8_t saturation;
  uint8_t hmirror;
  uint8_t vflip;
  uint8_t wb;      // 0 auto, 1 sunny, 2 cloudy, 3 office, 4 home
  uint8_t effect;  // 0 none, 1 negative, 2 grayscale, 3 red, 4 green, 5 blue, 6 sepia
  int8_t aeLevel;  // exposure level -2..2
};

// preview: 0 none, 1 hold drive servos at their center pulse, 2/3 grip min/max, 4/5 lift min/max (not stored)
struct __attribute__((packed)) RvServo {
  RvHdr h;
  uint16_t centerL, centerR;  // us, neutral pulse of the continuous rotation drive servos
  uint16_t gripMin, gripMax;  // us
  uint16_t liftMin, liftMax;  // us
  uint8_t gripRev, liftRev;
  uint8_t revL, revR;  // reverse the left / right drive servo
  int16_t trim;        // permille, >0 = left side faster (tank veers right)
  uint8_t preview;
};
