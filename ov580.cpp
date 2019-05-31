#include <iomanip>
#include <thread>

#include "pangolin/factory/factory_registry.h"
#include <pangolin/video/iostream_operators.h>
#include <ov580.h>
#include <ov580sensors.h>

// always open camera and then start triggering (otherwise the ov580 won't boot correctly)
// if you stop triggering cameras will keep streaming to stop it you need to put it in stdby
// if not in centered exposure mode, fps must be a multiple of the trigger, generally the same as
// the trigger

constexpr int MAX_UVC_DEVICES = 10;

namespace surreal {
using namespace pangolin;

bool Ov580Video::Init() {
  bool success = true;
  // stop streaming
  success &= sensors->StopStreaming();

  success &= sensors->Init();

  // Disable 580 ISP logic
  // NOTE - Can't disable both color interploation (i.e. demosaic) and
  //  denoise path, or images stop flowing.  Not sure why
  // Also... leaving in YUV output mode because can't figure out what representation
  //  is used when we turn this off.  Only the Y components have data
  success &= SetSystemRegister(0x80181000, 0x00);
  success &= SetSystemRegister(0x80181001, 0x21);
  success &= SetSystemRegister(0x80181002, 0x00);

  success &= SetSystemRegister(0x80181800, 0x00);
  success &= SetSystemRegister(0x80181801, 0x21);
  success &= SetSystemRegister(0x80181802, 0x00);

  // start streaming
  success &= sensors->StartStreaming();

  return success;
}

void Ov580Video::Start() {
  return uvc_video->Start();
}

void Ov580Video::Stop() {
  return uvc_video->Stop();
}

void Ov580Video::SetupDeviceProperties() {
  for (size_t i = 0; i < 2; ++i) {
    // Start out with uvc properties as the base
    device_properties[i] = GetVideoDeviceProperties(uvc_video.get());
    device_properties[i]["SerialNumber"] = (i == 0) ? sn_l : sn_r;
    device_properties[i]["PeriodUs"] = period_us;
    device_properties[i]["MaxExposureUs"] = sensors->GetMaxExposureUs();
    device_properties[i]["SensorModel"] = sensors->SensorModelString();
  }
}

void Ov580Video::SetupStreams() {
  // ov580 returns the two cameras merged side by side into a single frame
  // but returns GAY16LE as fmt. Let's mangle things and setup childs correctly.
  PixelFormat fmt = PixelFormatFromString("GRAY8");
  const size_t source_w = uvc_video->Streams()[0].Width();
  const size_t source_h = uvc_video->Streams()[0].Height();
  const size_t source_p = uvc_video->Streams()[0].Pitch();
  fmt.channel_bit_depth = 8;
  const StreamInfo si1(fmt, source_w, source_h, source_p, 0);
  const StreamInfo si2(fmt, source_w, source_h, source_p, (unsigned char*)(source_w));
  streams.push_back(si1);
  streams.push_back(si2);
}

std::vector<std::pair<std::string, std::string>>
Ov580Video::FindOv580Pairs(Ov580Sensors::SensorModel sm, const size_t width, const size_t height) {
  std::vector<std::pair<std::string, std::string>> result;

  Ov580Video video;
  video.sensor_model = sm;

  for (size_t device_id = 0; device_id < MAX_UVC_DEVICES; device_id++) {
    std::stringstream ss;
#if defined(WIN32)
    ss << "uvc:[vid=" << OV580_VID << ",pid=" << OV580_PID << ",num=" << device_id
       << ",size=" << width << "x" << height << ",fps=0]//";
#else
    ss << "v4l:[size=" << width << "x" << height << "]///dev/video" << device_id;
#endif
    std::unique_ptr<pangolin::VideoInterface> uvc_video;
    try {
      video.uvc_video =
          pangolin::FactoryRegistry<VideoInterface>::I().Open(pangolin::ParseUri(ss.str()));
    } catch (const pangolin::VideoExceptionNoKnownHandler&) {
      throw pangolin::VideoException("ov580 requires libuvc to be compiled in Pangolin.");
    } catch (const pangolin::VideoException&) {
      // In the process of opening devices to check their serial number we
      // might end up attempting to open a device that is already in use;
      // this is still an non valid interface but we need to catch the exception.
      video.uvc_video.reset();
    }

    if (video.uvc_video != nullptr) {
      video.sensors = Ov580Sensors::Create(sm, video, false);

      // Some sleep to give the cameras enough time to boot correctly.
      std::this_thread::sleep_for(std::chrono::milliseconds(200));

      if (video.sensors->CheckSensorId()) {
        std::string sn_l;
        std::string sn_r;
        if (video.sensors->GetSerials(sn_r, sn_l)) {
          result.push_back(std::pair<std::string, std::string>(sn_l, sn_r));
        }
      }
      video.uvc_video->Stop();
      video.uvc_video.reset();
    }
  }
  return result;
}

Ov580Video::Ov580Video(
    Ov580Sensors::SensorModel sm,
    std::string serial_number,
    int width,
    int height,
    uint32_t period,
    bool ext_trig,
    bool ce,
    uint32_t strobe_duration_us,
    Params& pars)
    : sensor_model(sm),
      centered_exposure(ce),
      external_trigger(ext_trig),
      transfer_bandwidth_gbps(0),
      period_us(period) {
  // Create 'Fake' cameras behind this filter driver so that video properties work out
  attached_cameras.reserve(2);
  for (size_t i = 0; i < 2; ++i) {
    attached_cameras.emplace_back(*this, i);
    attached_cameras_weak.emplace_back(&attached_cameras[i]);
  }

  // Create the uvc video object behind this ov580 driver.
  // The OV580 does not return proper serial numbers so as a ugly workaround
  // we open one by one each of the first MAX_UVC_DEVICES we find and try to match
  // the sensors serial number.

  bool matches = false;
  int device_id = 0;

  do {
    std::stringstream ss;
#if defined(WIN32)
    ss << "uvc:[vid=" << OV580_VID << ",pid=" << OV580_PID << ",num=" << device_id
       << ",size=" << width << "x" << height << ",fps=0]//";
#else
    ss << "v4l:[size=" << width << "x" << height << ",period=" << period_us << "]///dev/video"
       << device_id;
#endif

    try {
      uvc_video = pangolin::FactoryRegistry<VideoInterface>::I().Open(pangolin::ParseUri(ss.str()));
    } catch (const pangolin::VideoExceptionNoKnownHandler& e) {
      throw pangolin::VideoException("ov580 requires libuvc to be compiled in Pangolin.");
    } catch (const pangolin::VideoException& e) {
      // In the process of opening devices to check their serial number we
      // might end up attempting to open a device that is already in use;
      // this is still an non valid interface but we need to catch the exception.
      uvc_video.reset();
    }

    if (uvc_video) {
      sensors = Ov580Sensors::Create(sensor_model, *this, centered_exposure);
      // Some sleep to give the cameras enough time to boot correctly.
      std::this_thread::sleep_for(std::chrono::milliseconds(200));

      if (sensors->CheckSensorId()) {
        if (sensors->GetSerials(sn_r, sn_l)) {
          pango_print_info("ov580_serial%d:%s\n", device_id, (sn_l + sn_r).c_str());
          std::cout << std::flush;
        } else {
          pango_print_warn("ov580_serial%d:unable to read\n", device_id);
        }
      } else {
        matches = false;
      }

      // if the serial is empty we match the first uvc interface.
      if ((serial_number == (sn_l + sn_r)) || serial_number.empty()) {
        matches = true;
      } else {
        matches = false;
        uvc_video->Stop();
        uvc_video.reset();
      }
    }
    device_id++;
  } while (!matches && (device_id < MAX_UVC_DEVICES));

  if (matches) {
    if (!Init()) {
      throw pangolin::VideoException(
          "Not able to init ov580 cameras although they have the correct serial number.\n");
    }

    if (!sensors->SetPeriod(period_us)) {
      throw pangolin::VideoException(
          "Not able to set period on ov580 cameras although they correctly initialized.\n");
    }

    if (!sensors->EnableStrobe(strobe_duration_us)) {
      throw pangolin::VideoException(
          "Not able to enable ov580 strobe although cameras correctly initialized.\n");
    }

    if (external_trigger) {
      if (!sensors->EnableExternalTrigger()) {
        throw pangolin::VideoException(
            "Not able to enable ov580 external trigger although cameras correctly initialized.\n");
      }
    }

    // Add individual child streams for each camera.
    SetupStreams();

    // Create "streams" properties for each camera.
    SetupDeviceProperties();

    // Pass command line parameters to sensors
    SetDeviceParams(pars);

    // Turn off ov580 ISP algorithms.
    if (!DisableAAA()) {
      throw pangolin::VideoException(
          "Not able to disable ov580 ISP although cameras correctly initialized.\n");
    }
  } else {
    // release the video, just to be sure!
    uvc_video.reset();
    throw pangolin::VideoException(pangolin::FormatString(
        "Not able to find ov580 interface with serial number %s\n", serial_number.c_str()));
  }
}

void Ov580Video::SetDeviceParams(pangolin::Params& p) {
  for (Params::ParamMap::const_iterator it = p.params.begin(); it != p.params.end(); it++) {
    if (it->first == "transfer_bandwidth_gbps") {
      transfer_bandwidth_gbps = atof(it->second.c_str());
    } else {
      try {
        SetParameter(it->first, it->second);
      } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(OV580_RUNTIME_CMD_SLEEP));
    }
  }
}

void Ov580Video::UpdateFrameProperties(unsigned char* image) {
  double againl, againr;
  int expl_us, expr_us;

  sensors->GetFrameProperties(
      againl,
      againr,
      expl_us,
      expr_us,
      image,
      size_t(streams[0].Offset()),
      size_t(streams[1].Offset()));

  for (size_t i = 0; i < 2; ++i) {
    const uint32_t exp_us = (i == 0) ? expl_us : expr_us;
    // Use uvc properties as a base
    frame_properties[i] = GetVideoFrameProperties(uvc_video.get());
    frame_properties[i][PANGO_EXPOSURE_US] = exp_us;
    frame_properties[i][PANGO_ANALOG_GAIN] = (i == 0) ? againl : againr;
    //frame_properties[i][PANGO_HAS_LINE0_METADATA] = sensors->EmbeddedLine();
    if (transfer_bandwidth_gbps) {
      const int transfer_time_us = SizeBytes() / int64_t((transfer_bandwidth_gbps * 1E3) / 8.0);
      frame_properties[i][PANGO_ESTIMATED_CENTER_CAPTURE_TIME_US] =
          frame_properties[i][PANGO_HOST_RECEPTION_TIME_US].get<int64_t>() - int64_t(0.5 * exp_us) -
          transfer_time_us;
    }
  }
}

bool Ov580Video::GrabNext(unsigned char* image, bool wait) {
  bool r = uvc_video->GrabNext(image, wait);
  if (r) {
    UpdateFrameProperties(image);
  }
  return r;
}

bool Ov580Video::GrabNewest(unsigned char* image, bool wait) {
  bool r = uvc_video->GrabNewest(image, wait);
  if (r) {
    UpdateFrameProperties(image);
  }
  return r;
}

inline int Ov580Video::IoCtrlSetGet(int selector, unsigned char* buf, uint8_t len) {
  int ret;
  ret = dynamic_cast<pangolin::VideoUvcInterface*>(uvc_video.get())
            ->IoCtrl(OV580_UNIT_ID, selector, buf, len, pangolin::UVC_SET_CUR);
  if (ret < 0) {
    pango_print_warn("Ov580 IoCtrlSetGet error: %d\n", errno);
    return ret;
  }

  ret = dynamic_cast<pangolin::VideoUvcInterface*>(uvc_video.get())
            ->IoCtrl(OV580_UNIT_ID, selector, buf, len, pangolin::UVC_GET_CUR);
  if (ret < 0) {
    pango_print_warn("Ov580 IoCtrlSetGet error: %d\n", errno);
    return ret;
  }
  return 0;
}

bool Ov580Video::SetRegister(
    uint16_t sensor_add,
    Ov580Sensors::ModuleSelect side,
    uint8_t regVal,
    uint32_t sleep) {
  // Write left, right or both
  unsigned char SCCBCommands[2] = {0xa5, 0xa3};
  int SCCBStart = side == Ov580Sensors::ModuleSelect::RIGHT ? 1 : 0;
  int SCCBEnd = side == Ov580Sensors::ModuleSelect::LEFT ? 1 : 2;

  for (int SCCBIdx = SCCBStart; SCCBIdx < SCCBEnd; SCCBIdx++) {
    uint8_t value[OV580_PACKET_SIZE];
    memset(value, 0, OV580_PACKET_SIZE);

    value[0] = 0x50; // Command ID
    value[1] = SCCBCommands[SCCBIdx]; // Command type (A3 = SCCB0, A5 = SCCB1)
    value[2] = sensors->SCCBAddr(); // SCCB Slave id
    value[3] = 0x02; // Address width
    value[4] = 0x01; // Data width

    // Register address, big endian
    value[5] = 0x00;
    value[6] = 0x00;
    value[7] = (sensor_add >> 8) & 0xff;
    value[8] = sensor_add & 0xff;

    value[9] = 0x90; // Register number.  Why 0x90?
    value[10] = 0x01;

    value[16] = regVal;

    std::this_thread::sleep_for(std::chrono::microseconds(sleep));
    int ret =
        dynamic_cast<pangolin::VideoUvcInterface*>(uvc_video.get())
            ->IoCtrl(
                OV580_UNIT_ID, OV580_SELECTOR, value, OV580_PACKET_SIZE, pangolin::UVC_SET_CUR);
    if (ret < 0) {
      pango_print_warn("Ov580 SetRegister error: %d\n", errno);
      return false;
    }
  }

  return true;
}

bool Ov580Video::GetRegister(
    uint16_t sensor_add,
    Ov580Sensors::ModuleSelect side,
    uint8_t& result,
    uint32_t sleep) {
  PANGO_ASSERT(side != Ov580Sensors::ModuleSelect::BOTH);

  unsigned char SCCBCommands[2] = {0xa5, 0xa3};

  uint8_t value[OV580_PACKET_SIZE];
  memset(value, 0, OV580_PACKET_SIZE);

  value[0] = 0x51; // Command ID
  value[1] = SCCBCommands[(size_t)side]; // Command type (A3 = SCCB0, A5 = SCCB1)
  value[2] = sensors->SCCBAddr(); // SCCB Slave id
  value[3] = 0x02; // Address width
  value[4] = 0x01; // Data width

  // Register address, big endian
  value[5] = 0x00;
  value[6] = 0x00;
  value[7] = (sensor_add >> 8) & 0xff;
  value[8] = sensor_add & 0xff;

  value[9] = 0x90; // Register number.  Why 0x90?
  value[10] = 0x01;

  // 11 - 15 reserved

  int ret = 0;

  ret =
      dynamic_cast<pangolin::VideoUvcInterface*>(uvc_video.get())
          ->IoCtrl(OV580_UNIT_ID, OV580_SELECTOR, value, OV580_PACKET_SIZE, pangolin::UVC_SET_CUR);
  if (ret < 0) {
    pango_print_warn("Ov580 GetRegister error: %d\n", errno);
    return false;
  }

  std::this_thread::sleep_for(std::chrono::microseconds(sleep));

  value[2] = sensors->SCCBAddr() | 0x1; // SCCB Slave id
  ret =
      dynamic_cast<pangolin::VideoUvcInterface*>(uvc_video.get())
          ->IoCtrl(OV580_UNIT_ID, OV580_SELECTOR, value, OV580_PACKET_SIZE, pangolin::UVC_GET_CUR);
  if (ret < 0) {
    pango_print_warn("Ov580 GetRegister error: %d\n", errno);
    return false;
  }

  result = value[17];

  return true;
}

bool Ov580Video::SetSystemRegister(uint32_t system_addr, uint8_t regVal) {
  uint8_t value[OV580_PACKET_SIZE];
  memset(value, 0, OV580_PACKET_SIZE);

  value[0] = 0x50;
  value[1] = 0xa2;
  value[2] = sensors->SCCBAddr();
  value[3] = 0x04;
  value[4] = 0x01;

  // register address
  value[5] = (system_addr >> 24) & 0xff;
  value[6] = (system_addr >> 16) & 0xff;
  value[7] = (system_addr >> 8) & 0xff;
  value[8] = system_addr & 0xff;

  value[9] = 0x90;
  value[10] = 0x01;
  value[11] = 0x00;
  value[12] = 0x01;

  value[16] = regVal;

  std::this_thread::sleep_for(std::chrono::microseconds(OV580_NON_RUNTIME_CMD_SLEEP));
  int ret =
      dynamic_cast<pangolin::VideoUvcInterface*>(uvc_video.get())
          ->IoCtrl(OV580_UNIT_ID, OV580_SELECTOR, value, OV580_PACKET_SIZE, pangolin::UVC_SET_CUR);
  if (ret < 0) {
    pango_print_warn("Ov580 SetSystemRegister error: %d\n", errno);
    return false;
  }

  return true;
}

bool Ov580Video::GetSystemRegister(uint32_t system_addr, uint8_t& result) {
  uint8_t value[OV580_PACKET_SIZE];
  memset(value, 0, OV580_PACKET_SIZE);

  value[0] = 0x51;
  value[1] = 0xa2;
  value[2] = sensors->SCCBAddr();
  value[3] = 0x04;
  value[4] = 0x01;

  value[5] = (system_addr >> 24) & 0xff;
  value[6] = (system_addr >> 16) & 0xff;
  value[7] = (system_addr >> 8) & 0xff;
  value[8] = system_addr & 0xff;

  value[9] = 0x90;
  value[10] = 0x1;
  value[11] = 0;
  value[12] = 0x1;

  int ret =
      dynamic_cast<pangolin::VideoUvcInterface*>(uvc_video.get())
          ->IoCtrl(OV580_UNIT_ID, OV580_SELECTOR, value, OV580_PACKET_SIZE, pangolin::UVC_SET_CUR);
  if (ret < 0) {
    pango_print_warn("Ov580 GetSystemRegister error: %d\n", errno);
    return false;
  } else {
    std::this_thread::sleep_for(std::chrono::microseconds(OV580_NON_RUNTIME_CMD_SLEEP));
    value[2] = sensors->SCCBAddr() | 0x1; // SCCB Slave id
    ret = dynamic_cast<pangolin::VideoUvcInterface*>(uvc_video.get())
              ->IoCtrl(
                  OV580_UNIT_ID, OV580_SELECTOR, value, OV580_PACKET_SIZE, pangolin::UVC_SET_CUR);
    if (ret < 0) {
      pango_print_warn("Ov580 GetSystemRegister error: %d\n", errno);
      return false;
    } else {
      result = value[17];
      return true;
    }
  }
}

bool Ov580Video::GetParameter(const std::string& name, std::string& value) {
  // We read back from the left sensor only since we always set both sensors to be equal.
  bool success = true;
  if (name == "ExposureTime") {
    int exposureUs;
    success = sensors->GetExposure(exposureUs);
    if (success) {
      value = std::to_string(exposureUs);
    }
  } else if (name == "Gain") {
    float gain;
    success = sensors->GetGain(gain);
    if (success) {
      value = std::to_string(gain);
    }
  } else if (name == "BlackLevelTarget") {
    uint16_t target;
    success = sensors->GetBlackLevel(target);
    if (success) {
      value = std::to_string(target);
    }
  } else if (name == "strobe") {
    uint32_t target;
    success = sensors->GetStrobe(target);
    if (success) {
      value = std::to_string(target);
    }
  } else if (name == "MaxExposureUs") {
    value = std::to_string(sensors->GetMaxExposureUs());
  } else if (name == "sn_r") {
    value = sn_r;
  } else if (name == "sn_l") {
    value = sn_l;
  } else {
    pango_print_warn("Ov580 Unknown parameter %s\n", name.c_str());
    return false;
  }

  if (!success) {
    pango_print_warn("Ov580 getting parameter %s failed.\n", name.c_str());
    return false;
  }

  return true;
}

bool Ov580Video::SetParameter(const std::string& name, const std::string& value) {
  bool success = true;
  if (name == "ExposureTime") {
    int exposureUs = std::atoi(value.c_str());
    sensors->SetExposure(exposureUs);
  } else if (name == "Gain") {
    float gain = std::atof(value.c_str());
    sensors->SetGain(gain);
  } else if (name == "BlackLevelTarget") {
    uint8_t target = std::atoi(value.c_str());
    sensors->SetBlackLevel(target);
  } else if (name == "strobe") {
    uint32_t duration = std::atoi(value.c_str());
    sensors->EnableStrobe(duration);
  } else {
    pango_print_warn("Ov580 Unknown parameter %s\n", name.c_str());
    return false;
  }

  if (!success) {
    pango_print_warn("Ov580 setting parameter %s failed.\n", name.c_str());
    return false;
  }
  return true;
}

bool Ov580Video::DisableAAA() {
  bool success = SetSystemRegister(0x80181033, 0x00);
  success &= SetSystemRegister(0x80181833, 0x00);

  success &= sensors->DisableAAA();

  if (!success) {
    pango_print_warn("Ov580 Unable to disable AAA\n");
  }
  return success;
}

const std::vector<StreamInfo>& Ov580Video::Streams() const {
  return streams;
}

//PANGOLIN_REGISTER_FACTORY(Ov580Video) {
void RegisterOv580VideoFactory() {
  using namespace pangolin;

  struct Ov580VideoFactory : public FactoryInterface<VideoInterface> {
    std::unique_ptr<VideoInterface> Open(const Uri& uri) override {
      std::string serial_num = uri.Get<std::string>("sn", "");
      bool ext_trig = uri.Get<bool>("ExternalTrigger", false);
      bool centered_exposure = uri.Get<bool>("CenteredExposure", false);

      ImageDim dim;
      surreal::Ov580Sensors::SensorModel sensor_model;
      std::string cmdline_sensor_model_str = uri.Get<std::string>("SensorModel", "7251");

      // Parameter checking for the centered exposure case
      if (centered_exposure && !ext_trig) {
        pango_print_info("OV580: discarding CenteredExposure since ExternalTrigger is not set\n");
        centered_exposure = false;
      }
      if (uri.Contains("period") && uri.Contains("fps")) {
        pango_print_error("OV580: period and fps are specified! This is not allowed.\n");
        return 0;
      }

      if (centered_exposure && !(uri.Contains("period") || uri.Contains("fps"))) {
        pango_print_error("OV580: with CenteredExposure=1 period (or fps) must be specified!\n");
        return 0;
      }

      uint32_t period_us;
      uint32_t strobe_duration_us;

      if (cmdline_sensor_model_str == "7251" || cmdline_sensor_model_str == "7750") {
        if (cmdline_sensor_model_str == "7251") {
          sensor_model = surreal::Ov580Sensors::SensorModel::OV7251;
        } else {
          sensor_model = surreal::Ov580Sensors::SensorModel::OV7750;
        }
        dim = uri.Get<ImageDim>("size", ImageDim(640, 480));
        strobe_duration_us = uri.Get<uint32_t>("strobe", 100);
        period_us = 10000;
      } else if (
          cmdline_sensor_model_str == "9281" || cmdline_sensor_model_str == "9282" ||
          cmdline_sensor_model_str == "9782") {
        if (cmdline_sensor_model_str == "9782") {
          sensor_model = surreal::Ov580Sensors::SensorModel::OV9782;
        } else {
          sensor_model = surreal::Ov580Sensors::SensorModel::OV928x;
        }
        dim = uri.Get<ImageDim>("size", ImageDim(1280, 800));
        if ((dim.x == 1280) && (dim.y == 800)) {
          period_us = 10000;
        } else {
          period_us = 5000;
        }
        strobe_duration_us = uri.Get<uint32_t>("strobe", 100);
      } else {
        throw pangolin::VideoException(
            "Ov580 sensor model not recognized " + cmdline_sensor_model_str);
      }

      if (!uri.Contains("fps")) {
        period_us = uri.Get<uint32_t>("period", period_us);
      } else {
        period_us = uint32_t(1e6f / uri.Get<uint32_t>("fps", 100));
      }

      Params params;
      // we always need a starting exposure
      if (!uri.Contains("ExposureTime")) {
        params.Set("ExposureTime", "5000");
      }

      if (!uri.Contains("Gain")) {
        params.Set("Gain", "1");
      }

      for (Params::ParamMap::const_iterator it = uri.params.begin(); it != uri.params.end(); it++) {
        if ((it->first != "sn") && (it->first != "period") && (it->first != "fps") &&
            (it->first != "ExternalTrigger") && (it->first != "CenteredExposure") &&
            (it->first != "SensorModel") && (it->first != "strobe") && (it->first != "size")) {
          params.Set(it->first, it->second);
        }
      }

      return std::unique_ptr<VideoInterface>(new surreal::Ov580Video(
          sensor_model,
          serial_num,
          dim.x,
          dim.y,
          period_us,
          ext_trig,
          centered_exposure,
          strobe_duration_us,
          params));
    }
  };

  FactoryRegistry<VideoInterface>::I().RegisterFactory(
      std::make_shared<Ov580VideoFactory>(), 15, "ov580");
}
} // namespace surreal
