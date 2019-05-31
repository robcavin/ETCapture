#pragma once

#include <ov580sensors.h>
#include <vector>

namespace surreal {
class Ov7251pair : public Ov580Sensors {
  constexpr static uint16_t CHIPID = 0x7750;
  constexpr static uint16_t CHIPID_L = 0x300a;
  constexpr static uint16_t CHIPID_H = 0x300b;
  constexpr static uint16_t OTP_START = 0x3d00;
  constexpr static uint16_t OTP_PLL = 0x100;
  constexpr static uint8_t OTP_PLL_ON = 0x1;
  constexpr static size_t OTP_SIZE = 32;
  constexpr static uint16_t OTP_READ = 0x3d81;
  constexpr static uint16_t OTP_READ_CMD = 0x01;
  constexpr static uint8_t SCLKFreqMHZ = 48;
  constexpr static uint16_t SC_MODE_SELECT = 0x0100;
  constexpr static uint8_t SC_MODE_SELECT_STREAM = 0x01;
  constexpr static uint8_t SC_MODE_SELECT_STDBY = 0x00;
  constexpr static uint16_t TIMING_REG_23 = 0x3823;
  constexpr static uint16_t ANA_CORE_6 = 0x3666;
  constexpr static uint8_t EL_ANALOGGAIN_ID_R = 3;
  constexpr static uint8_t EL_ANALOGGAIN_ID_G = 3;
  constexpr static uint8_t EL_ANALOGGAIN_ID_B = 3;
  constexpr static uint8_t EL_EXPTIMEH_ID = 7;
  constexpr static uint8_t EL_EXPTIMEL_ID = 8;
  constexpr static int MAX_CONSECUTIVE_ZEROS_IN_SERIAL = 10;

  constexpr static uint8_t SCCBADDR_7251 = 0xC0;
  constexpr static uint8_t SCCBADDR_7750 = 0xC0;

  constexpr static bool USE_EMBEDDED_LINE = true;

 public:
  virtual ~Ov7251pair();
  bool CheckSensorId() override;
  bool Init() override;
  bool StopStreaming() override;
  bool StartStreaming() override;
  // Note: ``strobe_duration_us`` must be shorter than ``period_us``, otherwise the behaviour will
  // be undefined.
  bool EnableStrobe(uint32_t strobe_duration_us) override;
  bool GetStrobe(uint32_t& strobe_duration_us) override;
  bool EnableExternalTrigger() override;
  bool GetSerials(std::string& sn_r, std::string& sn_l) override;
  bool SetPeriod(uint32_t period_us) override;
  bool DisableAAA() override;
  uint32_t GetMaxExposureUs() override;
  bool SetExposure(int exposureUs) override;
  bool SetGain(float gain) override;
  bool SetBlackLevel(uint16_t target) override;
  bool GetExposure(int& exposureUs) override;
  bool GetGain(float& gain) override;
  std::string SensorModelString() override;
  bool GetBlackLevel(uint16_t& target) override;
  void GetFrameProperties(
      double& againl,
      double& againr,
      int& expl_us,
      int& expr_us,
      unsigned char* image,
      size_t offset0,
      size_t offset1) override;

 public:
  // if testmode is true, then CheckSensorId() fails to verify the exptected sensor model matches
  // the given sensor model, the constructor will continue to set the registers, this is purely for
  // the purpose of testing new sensors feature requested by Renzo
  Ov7251pair(
      Ov580Video& ov580interface,
      bool centered_exposure,
      Ov580Sensors::SensorModel type,
      bool testmode = false);

 private:
  bool ReadOTP(ModuleSelect sensor, std::vector<uint8_t>& bytes);
  bool GetSensorSerial(ModuleSelect sensor, std::string& serial);
};
} // namespace surreal
