#pragma once

#include <pangolin/factory/factory_registry.h>
#include <pangolin/pangolin.h>
#include <pangolin/video/video.h>
#include <pangolin/video/video_interface.h>
#include <ov580sensors.h>
#include <queue>

namespace surreal {
const static std::string OV580_VID = "0x05a9";
const static std::string OV580_PID = "0x0581";
constexpr int MAX_CONSECUTIVE_ZEROS = 10;
constexpr int OV580_UNIT_ID = 4;
constexpr int OV580_SELECTOR = 2;
constexpr size_t OV580_PACKET_SIZE = 384;
constexpr size_t OV580_NON_RUNTIME_CMD_SLEEP = 1000;
constexpr size_t OV580_RUNTIME_CMD_SLEEP = 150;

class PANGOLIN_EXPORT Ov580Video : public pangolin::VideoInterface,
                                   public pangolin::VideoFilterInterface,
                                   public pangolin::GenicamVideoInterface {
 public:
  static std::vector<std::pair<std::string, std::string>> FindOv580Pairs(
      Ov580Sensors::SensorModel sensor_model,
      const size_t width = 640,
      const size_t height = 480);

  Ov580Video(
      Ov580Sensors::SensorModel sensor_model,
      std::string serial_number,
      int width,
      int height,
      uint32_t period_us,
      bool ext_trig,
      bool centered_exposure,
      uint32_t strobe_duration_us,
      pangolin::Params& p);

  ~Ov580Video() {}

  //! Implement VideoInput::Start()
  void Start() override;

  //! Implement VideoInput::Stop()
  void Stop() override;

  //! Implement VideoInput::SizeBytes()
  size_t SizeBytes() const override {
    PANGO_ASSERT(uvc_video);
    return uvc_video->SizeBytes();
  }

  //! Implement VideoInput::GrabNext()
  bool GrabNext(unsigned char* image, bool wait = true) override;

  //! Implement VideoInput::GrabNewest()
  bool GrabNewest(unsigned char* image, bool wait = true) override;

  const std::vector<pangolin::StreamInfo>& Streams() const override;

  std::vector<VideoInterface*>& InputStreams() override {
    return attached_cameras_weak;
  }

  bool SetSystemRegister(uint32_t system_addr, uint8_t regVal);

  bool GetSystemRegister(uint32_t system_addr, uint8_t& result);

  bool GetRegister(
      uint16_t sensor_add,
      Ov580Sensors::ModuleSelect side,
      uint8_t& result,
      uint32_t sleep = OV580_NON_RUNTIME_CMD_SLEEP);

  bool SetRegister(
      uint16_t sensor_add,
      Ov580Sensors::ModuleSelect side,
      uint8_t regVal,
      uint32_t sleep = OV580_NON_RUNTIME_CMD_SLEEP);

  inline int IoCtrlSetGet(int selector, unsigned char* buf, uint8_t len);

  bool SetExposure(uint32_t exposure_us);

  size_t CameraCount() const override {
    return 2;
  }

  bool GetParameter(const std::string& name, std::string& value) override;

  bool SetParameter(const std::string& name, const std::string& value) override;

 private:
  struct Ov580VideoCamera : public pangolin::VideoInterface,
                            public pangolin::VideoPropertiesInterface {
    Ov580VideoCamera(Ov580Video& ov580, size_t camera_index)
        : ov580(ov580), camera_index(camera_index) {}

    size_t SizeBytes() const {
      throw pangolin::VideoException("Dummy Method should not be called");
    }
    const std::vector<pangolin::StreamInfo>& Streams() const {
      throw pangolin::VideoException("Dummy Method should not be called");
    }
    void Start() {
      throw pangolin::VideoException("Dummy Method should not be called");
    }
    void Stop() {
      throw pangolin::VideoException("Dummy Method should not be called");
    }

    bool GrabNext(unsigned char*, bool) {
      throw pangolin::VideoException("Dummy Method should not be called");
    }

    bool GrabNewest(unsigned char*, bool) {
      throw pangolin::VideoException("Dummy Method should not be called");
    }

    const picojson::value& DeviceProperties() const {
      return ov580.device_properties[camera_index];
    }

    const picojson::value& FrameProperties() const {
      return ov580.frame_properties[camera_index];
    }

    Ov580Video& ov580;
    size_t camera_index;
  };

  // private default constructor for search to use.
  // You can't make Ov580Sensors without Ov580Video :-(
  Ov580Video() {}

  std::unique_ptr<pangolin::VideoInterface> uvc_video;
  std::vector<Ov580VideoCamera> attached_cameras;
  std::vector<VideoInterface*> attached_cameras_weak;

  Ov580Sensors::SensorModel sensor_model;
  std::vector<pangolin::StreamInfo> streams;
  bool centered_exposure;
  bool external_trigger;
  double transfer_bandwidth_gbps;

  picojson::value device_properties[2];
  picojson::value frame_properties[2];
  std::string sn_r;
  std::string sn_l;
  uint32_t period_us;

  std::unique_ptr<Ov580Sensors> sensors;

  bool Init();
  void SetupDeviceProperties();
  void SetupStreams();
  void SetDeviceParams(pangolin::Params& p);
  bool DisableAAA();
  void UpdateFrameProperties(unsigned char* image);
};

    void RegisterOv580VideoFactory();
} // namespace surreal
