#pragma once

#include <stdint.h>

#include <memory>
#include <string>

namespace surreal {
class Ov580Video;

class Ov580Sensors {
 public:
  enum SensorModel { OV7750, OV7251, OV928x, OV9782 };

  enum ModuleSelect { LEFT = 0, RIGHT = 1, BOTH = 2 };

 public:
  virtual ~Ov580Sensors() {}
  virtual bool CheckSensorId() = 0;
  virtual bool Init() = 0;
  virtual bool StopStreaming() = 0;
  virtual bool StartStreaming() = 0;
  virtual bool EnableStrobe(uint32_t strobe_duration_us) = 0;
  virtual bool GetStrobe(uint32_t& strobe_duration_us) = 0;
  virtual bool EnableExternalTrigger() = 0;
  virtual bool GetSerials(std::string& sn_r, std::string& sn_l) = 0;
  virtual bool SetPeriod(uint32_t period_us) = 0;
  virtual bool DisableAAA() = 0;
  virtual uint32_t GetMaxExposureUs() = 0;
  virtual bool SetExposure(int exposureUs) = 0;
  virtual bool SetGain(float gain) = 0;
  virtual bool SetBlackLevel(uint16_t target) = 0;
  virtual bool GetExposure(int& exposureUs) = 0;
  virtual bool GetGain(float& gain) = 0;
  virtual std::string SensorModelString() = 0;
  virtual bool GetBlackLevel(uint16_t& target) = 0;
  virtual bool EmbeddedLine() {
    return embedded_line;
  }
  virtual void GetFrameProperties(
      double& againl,
      double& againr,
      int& expl_us,
      int& expr_us,
      unsigned char* image,
      size_t offset0,
      size_t offset1) = 0;
  static std::unique_ptr<Ov580Sensors>
  Create(SensorModel type, Ov580Video& ov580interface, bool centered_exposure);
  void DumpRegsToFile(std::string filename);
  uint8_t SCCBAddr() {
    return sccb_addr;
  }

 protected:
  Ov580Sensors(
      Ov580Video& ov580interface,
      bool centered_exp,
      SensorModel type,
      bool testmode = false)
      : max_exp_rows(0),
        vi(ov580interface),
        centered_exposure(centered_exp),
        embedded_line(false),
        test_mode(testmode) {
    sensor_model = type;
  }

  uint16_t hts;
  double rowTime;
  uint32_t max_exp_rows;
  Ov580Video& vi;
  bool centered_exposure;
  bool embedded_line;
  uint32_t exposure_us;
  float again;
  uint8_t sccb_addr;
  SensorModel sensor_model;

  bool test_mode;
};
} // namespace surreal
