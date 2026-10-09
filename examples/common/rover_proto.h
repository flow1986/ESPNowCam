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
  RV_PONG = 4,   // tank -> controller
  RV_SERVO = 5   // controller -> tank, servo calibration (resent every second)
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

// preview: 0 none, 1 hold drive servos at their center pulse, 2/3 grip min/max, 4/5 lift min/max
struct __attribute__((packed)) RvServo {
  RvHdr h;
  uint16_t centerL, centerR;  // us, neutral pulse of the continuous rotation drive servos
  uint16_t gripMin, gripMax;  // us
  uint16_t liftMin, liftMax;  // us
  uint8_t gripRev, liftRev;
  uint8_t preview;
};
