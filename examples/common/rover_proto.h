// Packets shared by the CYD rover controller and the S3 CAM rover tank.
// They travel as payload of ESPNowCam sendData() (little endian, packed).
// Video frames are JPEG (start with 0xFF), control packets start with RV_MAGIC.
#pragma once
#include <stdint.h>

#define RV_MAGIC 0xA5
#define RV_MAX_TANKS 7  // tank id == ESP-NOW channel == DIP switch value (1..7)

enum RvType : uint8_t {
  RV_DRIVE = 1,  // controller -> tank
  RV_CAM = 2,    // controller -> tank, camera settings
  RV_PING = 3,   // controller -> tank, tank 0 = any tank on this channel
  RV_PONG = 4    // tank -> controller
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
};
