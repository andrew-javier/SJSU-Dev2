#pragma once

#include <cstdint>
#include <cstdio>

#include "L1_Drivers/i2c.hpp"

class AccelerometerInterface
{
 public:
  virtual bool Init()                                 = 0;
  virtual int16_t GetX()                              = 0;
  virtual int16_t GetY()                              = 0;
  virtual int16_t GetZ()                              = 0;
  virtual float GetPitch()                            = 0;
  virtual float GetRoll()                             = 0;
  virtual int GetFullScaleRange()                     = 0;
  virtual void SetFullScaleRange(uint8_t range_value) = 0;
};

class Accelerometer : public AccelerometerInterface
{
 public:
  uint16_t const kDataOffset         = 16;
  float const kRadiansToDegree       = 180 / 3.14f;
  uint8_t const kWhoAmIExpectedValue = 0x2a;
  uint8_t const kMsbShift            = 8;
  size_t const kEightBitLength       = 1;
  size_t const kSixteenBitLength     = 2;

  int gValue[4]         = { 2, 4, 8, -1 };
  uint8_t sendValue[16] = { 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
                            0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

  enum RegisterMap : uint8_t
  {
    status      = 0x00,
    x           = 0x01,
    y           = 0x03,
    z           = 0x05,
    who_am_i    = 0x0d,
    data_config = 0x0e
  };

  explicit constexpr Accelerometer(uint8_t address = 0x1c)
      : i2c_(&accelerometer_device_),
        accelerometer_device_(),
        accelerometer_address_(address)
  {
  }
  bool Init() override
  {
    i2c_->Initialize();
    i2c_->Write(accelerometer_address_, { 0x2A, 0x01 });
    uint8_t WhoAmIReceivedValue;
    uint8_t IdentityRegister = RegisterMap::who_am_i;
    i2c_->WriteThenRead(accelerometer_address_, &IdentityRegister,
                        kEightBitLength, &WhoAmIReceivedValue, kEightBitLength);
    return (WhoAmIReceivedValue == kWhoAmIExpectedValue);
  }
  int16_t GetX() override
  {
    int tiltreadings;
    int16_t xtilt;
    uint8_t kXVal[2];
    uint8_t XReg = RegisterMap::x;
    i2c_->WriteThenRead(accelerometer_address_, &XReg, kEightBitLength, kXVal,
                        kSixteenBitLength);
    tiltreadings = (kXVal[0] << kMsbShift) | kXVal[1];
    xtilt        = static_cast<int16_t>(tiltreadings);
    return xtilt;
  }
  int16_t GetY() override
  {
    int tiltreadings;
    int16_t ytilt;
    uint8_t kYVal[2];
    uint8_t YReg = RegisterMap::y;
    i2c_->WriteThenRead(accelerometer_address_, &YReg, kEightBitLength, kYVal,
                        2);
    tiltreadings = (kYVal[0] << kMsbShift) | kYVal[1];
    ytilt        = static_cast<int16_t>(tiltreadings);
    return ytilt;
  }
  int16_t GetZ() override
  {
    int tiltreadings;
    int16_t ztilt;
    uint8_t kZVal[2];
    uint8_t ZReg = RegisterMap::z;
    i2c_->WriteThenRead(accelerometer_address_, &ZReg, kEightBitLength, kZVal,
                        2);
    tiltreadings = (kZVal[0] << kMsbShift) | kZVal[1];
    ztilt        = static_cast<int16_t>(tiltreadings);
    return ztilt;
  }
  float GetPitch() override
  {
    int x                   = GetX();
    int y                   = GetY();
    int z                   = GetZ();
    float kPitchNumerator   = x * -1;
    float kPitchDenominator = sqrtf((y * y) + (z * z));
    float pitch = atan2f(kPitchNumerator, kPitchDenominator) * kRadiansToDegree;
    return pitch;
  }
  float GetRoll() override
  {
    int y = GetY();
    int z = GetZ();
    return (atan2f(y, z) * kRadiansToDegree);
  }
  int GetFullScaleRange() override
  {
    uint8_t configReg = RegisterMap::data_config;
    uint8_t FullScaleValue;
    i2c_->WriteThenRead(accelerometer_address_, &configReg, kEightBitLength,
                        &FullScaleValue, kEightBitLength);
    FullScaleValue &= 0x03;
    int range = gValue[FullScaleValue];
    return range;
  }
  void SetFullScaleRange(uint8_t range_value) override
  {
    range_value &= 0x0f;
    uint8_t configReg                    = RegisterMap::data_config;
    uint8_t sendRange                    = sendValue[range_value];
    uint8_t fullScaleRangeWriteBuffer[2] = { configReg, sendRange };
    i2c_->Write(accelerometer_address_, fullScaleRangeWriteBuffer,
                kSixteenBitLength);
  }
  virtual ~Accelerometer() {}

 private:
  I2cInterface * i2c_;
  I2c accelerometer_device_;
  uint8_t accelerometer_address_;
};
