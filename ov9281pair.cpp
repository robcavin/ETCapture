#include <ov9281pair.h>
#include <ov580.h>
#include <bitset>
#include <iomanip>
#include <thread>
namespace surreal {
Ov9281pair::Ov9281pair(
    Ov580Video& ov580interface,
    bool centered_exp,
    SensorModel type,
    bool testmode)
    : Ov580Sensors(ov580interface, centered_exp, type, testmode) {
  if (type == surreal::Ov580Sensors::SensorModel::OV9782)
    sccb_addr = SCCBADDR_9782;
  else
    sccb_addr = SCCBADDR_928x;
}

Ov9281pair::~Ov9281pair() {}

bool Ov9281pair::Init() {
  uint8_t hi = 0;
  uint8_t lo = 0;

  if (!CheckSensorId()) {
    if (!test_mode) {
      pango_print_warn("Ov580:9281 Unexpected sensor ID, stop setting up any registers!");
      return false;
    } else {
      pango_print_warn("Test mode: Ov580:9281 Unexpected sensor ID Will try to keep going...");
    }
  }

  bool success = vi.GetRegister(0x380c, ModuleSelect::LEFT, hi);
  success &= vi.GetRegister(0x380d, ModuleSelect::LEFT, lo);
  hts = hi << 8 | lo;
  rowTime = double(hts) / SCLKFreqMHZ;

  // enable embedded line as first line of frame
  if (USE_EMBEDDED_LINE) {
    embedded_line = vi.SetRegister(0x4814, ModuleSelect::BOTH, 0x2b);
    embedded_line = vi.SetRegister(0x4307, ModuleSelect::BOTH, 0x31);
    embedded_line = vi.SetRegister(0x366f, ModuleSelect::BOTH, 0x1a);
    embedded_line = vi.SetRegister(0x3662, ModuleSelect::BOTH, 0x05);
  }

  return success;
}

bool Ov9281pair::StopStreaming() {
  return vi.SetRegister(SC_MODE_SELECT, ModuleSelect::BOTH, SC_MODE_SELECT_STDBY);
}

bool Ov9281pair::StartStreaming() {
  return vi.SetRegister(SC_MODE_SELECT, ModuleSelect::BOTH, SC_MODE_SELECT_STREAM);
}

bool Ov9281pair::EnableStrobe(uint32_t strobe_duration_us) {
  // for now we assume stobe is setup only once and never changes at runtime.
  bool success = true;

  // Set the strobe timing step to 1 us (80 system clock cycles at 80 Mhz)
  constexpr uint16_t stepSizeInUs = 1;
  uint16_t const stepSizeInSysClk = (uint16_t)round(stepSizeInUs * SCLKFreqMHZ);
  success &= vi.SetRegister(0x392D, ModuleSelect::BOTH, uint8_t((stepSizeInSysClk >> 8) & 0xFF));
  success &= vi.SetRegister(0x392E, ModuleSelect::BOTH, uint8_t(stepSizeInSysClk & 0xFF));

  for (auto side : {ModuleSelect::LEFT, ModuleSelect::RIGHT}) {
    // Enable manual control of strobe timing step
    success &= vi.SetRegister(0x392F, side, 0x08);

    // Set Strobe pin to output
    success &= vi.SetRegister(0x3006, side, 0x08);

    if (strobe_duration_us == 0) {
      // Force strobe output value to low
      success &= vi.SetRegister(0x3009, side, 0x00);
      // Make sure strobe is not active for any frames
      success &= vi.SetRegister(0x3920, ModuleSelect::BOTH, 0x00);
    } else {
      // Make sure strobe is active far all frames
      success &= vi.SetRegister(0x3920, ModuleSelect::BOTH, 0xFF);
    }
  }

  // Compute strobe duration register value
  uint32_t strobeDurationInSteps = strobe_duration_us / stepSizeInUs;

  // Set strobe delay, with positive shift
  success &= vi.SetRegister(0x3921, ModuleSelect::BOTH, 0x00);
  success &= vi.SetRegister(0x3922, ModuleSelect::BOTH, 0x00);
  success &= vi.SetRegister(0x3923, ModuleSelect::BOTH, 0x00);
  success &= vi.SetRegister(0x3924, ModuleSelect::BOTH, 0x00);

  // Set strobe duration
  success &= vi.SetRegister(0x3925, ModuleSelect::BOTH, (strobeDurationInSteps >> 24) & 0xFF);
  success &= vi.SetRegister(0x3926, ModuleSelect::BOTH, (strobeDurationInSteps >> 16) & 0xFF);
  success &= vi.SetRegister(0x3927, ModuleSelect::BOTH, (strobeDurationInSteps >> 8) & 0xFF);
  success &= vi.SetRegister(0x3928, ModuleSelect::BOTH, strobeDurationInSteps & 0xFF);

  return success;
}

bool Ov9281pair::GetStrobe(uint32_t& strobe_duration_us) {
  // Get strobe duration; fetch from left module, assuming our code always
  // puts the same values in both.
  constexpr uint16_t stepSizeInUs = 1;
  bool success = true;

  union {
    uint8_t result[sizeof(uint32_t)];
    uint32_t value;
  } converter;

  success &=
      vi.GetRegister(0x3925, ModuleSelect::LEFT, converter.result[3], OV580_RUNTIME_CMD_SLEEP);
  success &=
      vi.GetRegister(0x3926, ModuleSelect::LEFT, converter.result[2], OV580_RUNTIME_CMD_SLEEP);
  success &=
      vi.GetRegister(0x3927, ModuleSelect::LEFT, converter.result[1], OV580_RUNTIME_CMD_SLEEP);
  success &=
      vi.GetRegister(0x3928, ModuleSelect::LEFT, converter.result[0], OV580_RUNTIME_CMD_SLEEP);

  strobe_duration_us = converter.value * stepSizeInUs;
  return success;
}

bool Ov9281pair::EnableExternalTrigger() {
  bool success = true;
  // enable FSIN
  success &= vi.SetRegister(ANA_CORE_6, ModuleSelect::BOTH, 0x00);
  success &= vi.SetRegister(TIMING_REG_23, ModuleSelect::BOTH, 0x30);

  return success;
}

bool Ov9281pair::CheckSensorId() {
  uint8_t hi = 0;
  uint8_t lo = 0;
  bool success = vi.GetRegister(CHIPID_L, ModuleSelect::LEFT, hi);
  success &= vi.GetRegister(CHIPID_H, ModuleSelect::LEFT, lo);

  uint16_t sensorId = (hi << 8) | lo;

  if (sensorId != CHIPID) {
    return false;
  }

  success &= vi.GetRegister(CHIPID_L, ModuleSelect::RIGHT, hi);
  success &= vi.GetRegister(CHIPID_H, ModuleSelect::RIGHT, lo);
  hi = (hi << 8) | lo;

  if (sensorId != CHIPID) {
    return false;
  }

  return true;
}

bool Ov9281pair::GetSerials(std::string& sn_r, std::string& sn_l) {
  uint8_t hi = 0;
  uint8_t lo = 0;
  bool success = vi.GetRegister(CHIPID_L, ModuleSelect::LEFT, hi);
  success &= vi.GetRegister(CHIPID_H, ModuleSelect::LEFT, lo);

  uint16_t sensorId = (hi << 8) | lo;

  if (sensorId != CHIPID) {
    return false;
  }

  success &= vi.GetRegister(CHIPID_L, ModuleSelect::RIGHT, hi);
  success &= vi.GetRegister(CHIPID_H, ModuleSelect::RIGHT, lo);
  hi = (hi << 8) | lo;

  if (sensorId != CHIPID) {
    return false;
  }

  return GetSensorSerial(ModuleSelect::RIGHT, sn_r) && GetSensorSerial(ModuleSelect::LEFT, sn_l);
}

bool Ov9281pair::SetPeriod(uint32_t period_us) {
  StopStreaming();
  bool success = true;
  max_exp_rows = uint32_t((float(period_us) / rowTime) - 20);

  // in centered exposure mode we are nopt quite able to achieve 1/fps
  // experimentally we detremined that 0.75 / fps works
  if (centered_exposure) {
    max_exp_rows = (max_exp_rows * 2) / 3;
  }
  pango_print_info("Ov580:9281 max exposure %dus\n", int(max_exp_rows * rowTime));

  // note that centered_exposure -> external_trigger.
  if (!centered_exposure) {
    // horizontal time
    int vts = (SCLKFreqMHZ * period_us) / (hts);

    success &= vi.SetRegister(0x380e, ModuleSelect::BOTH, (vts >> 8) & 0xFF);
    success &= vi.SetRegister(0x380f, ModuleSelect::BOTH, vts & 0xFF);

    if (!success)
      pango_print_warn("Ov580:9281 not able to set FPS.\n");
  }
  StartStreaming();

  return success;
}

bool Ov9281pair::ReadOTP(ModuleSelect sensor, std::vector<uint8_t>& bytes) {
  bool success = true;
  // configure PLL
  success &= vi.SetRegister(OTP_PLL, sensor, OTP_PLL_ON);
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  success &= vi.SetRegister(0x3d18, sensor, 0x1);
  std::this_thread::sleep_for(std::chrono::milliseconds(15));

  // clear OTP registers, as per datsheet 4.5.4
  for (size_t offset = 0; offset < OTP_SIZE; ++offset) {
    success &= vi.SetRegister(OTP_START + offset, sensor, 0x00);
  }
  // trigger OTP readout, wait 15ms as per datasheet.
  success &= vi.SetRegister(OTP_READ, sensor, OTP_READ_CMD);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // read out OTP from registers
  for (size_t offset = 0; offset < OTP_SIZE; ++offset) {
    uint8_t val = 0;
    if (vi.GetRegister(OTP_START + offset, sensor, val)) {
      bytes.push_back(val);
    } else {
      success &= false;
    }
  }

  return success;
}

bool Ov9281pair::GetSensorSerial(ModuleSelect sensor, std::string& s) {
  bool success = true;
  std::vector<uint8_t> bytes;
  // often the OTP read fails and returns garbage
  // we observed that goos reads have the last byte = 1
  // and the remaining bytes are not all equal to 0.
  int cnt = 5;
  bool too_many_zeros = false;
  int consecutive_zeros_cnt = 0;

  do {
    bytes.clear();
    success = ReadOTP(sensor, bytes);
    cnt--;

    // Serial number is only the first 16 bytes...
    bytes.resize(16);
    too_many_zeros = false;
    consecutive_zeros_cnt = 0;
    for (const auto& byte : bytes) {
      consecutive_zeros_cnt = (byte == 0) ? consecutive_zeros_cnt + 1 : 0;
      if (consecutive_zeros_cnt > MAX_CONSECUTIVE_ZEROS_IN_SERIAL)
        too_many_zeros = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  } while (too_many_zeros && (cnt > 0));

  if ((cnt == 0) || !success) {
    s = "";
    return false;
  } else {
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (const auto& byte : bytes)
      ss << std::setw(2) << static_cast<unsigned>(byte);
    s = ss.str();

    return true;
  }
}

bool Ov9281pair::DisableAAA() {
  // disable AWB
  bool success = vi.SetRegister(0x3400, ModuleSelect::BOTH, 0x04);
  success &= vi.SetRegister(0x3401, ModuleSelect::BOTH, 0x00);
  success &= vi.SetRegister(0x3402, ModuleSelect::BOTH, 0x04);
  success &= vi.SetRegister(0x3403, ModuleSelect::BOTH, 0x00);
  success &= vi.SetRegister(0x3404, ModuleSelect::BOTH, 0x04);
  success &= vi.SetRegister(0x3405, ModuleSelect::BOTH, 0x00);
  success &= vi.SetRegister(0x3406, ModuleSelect::BOTH, 0x01);

  // manual gain and exposure
  success &= vi.SetRegister(0x3503, ModuleSelect::BOTH, 0x03);

  return success;
}

void Ov9281pair::GetFrameProperties(
    double& againl,
    double& againr,
    int& expl_us,
    int& expr_us,
    unsigned char* image,
    size_t offset0,
    size_t offset1) {
  if (embedded_line) {
    unsigned char* row = image + offset0;

    expl_us = ((int(row[EL_EXPTIMEH_ID] & EL_EXPTIMEH_MASK) << EL_EXPTIMEH_SHIFT_L) +
               (int(row[EL_EXPTIMEM_ID] & EL_EXPTIMEM_MASK) << EL_EXPTIMEM_SHIFT_L) +
               (int(row[EL_EXPTIMEL_ID] & EL_EXPTIMEL_MASK) >> EL_EXPTIMEL_SHIFT_R)) *
        rowTime;
    const int iagainl =
        (int(row[EL_ANALOGGAINH_ID] & EL_ANALOGGAINH_MASK) << EL_ANALOGGAINH_SHIFT_L) +
        (int(row[EL_ANALOGGAINL_ID] & EL_ANALOGGAINL_MASK) >> EL_ANALOGGAINL_SHIFT_R);
    againl = iagainl / 16.0;

    // update pointer to parse embedded line of second image.
    row = image + offset1;
    expr_us = ((int(row[EL_EXPTIMEH_ID] & EL_EXPTIMEH_MASK) << EL_EXPTIMEH_SHIFT_L) +
               (int(row[EL_EXPTIMEM_ID] & EL_EXPTIMEM_MASK) << EL_EXPTIMEM_SHIFT_L) +
               (int(row[EL_EXPTIMEL_ID] & EL_EXPTIMEL_MASK) >> EL_EXPTIMEL_SHIFT_R)) *
        rowTime;
    const int iagainr =
        (int(row[EL_ANALOGGAINH_ID] & EL_ANALOGGAINH_MASK) << EL_ANALOGGAINH_SHIFT_L) +
        (int(row[EL_ANALOGGAINL_ID] & EL_ANALOGGAINL_MASK) >> EL_ANALOGGAINL_SHIFT_R);
    againr = iagainr / 16.0;

    if (againl < 0.5 || againr < 0.5 || expl_us < 1 || expr_us < 1) {
      pango_print_warn(
          "Ov580: 9281 grab error: Corrupted Frame! Gains: %.2f / %.2f Exposures %d / %d\n",
          againl,
          againr,
          expr_us,
          expl_us);
      expr_us = expl_us = 0;
      againl = againr = 0;
    }
  } else {
    // We do not have an embedded line, return shadowed values
    againl = again;
    againr = again;
    expl_us = exposure_us;
    expr_us = exposure_us;
  }
}

bool Ov9281pair::SetExposure(int exposureUs) {
  bool success = true;
  if (exposureUs < rowTime)
    exposureUs = rowTime;
  // horizontal time low 4 bits are fraction bits
  uint32_t numFractionalRows = (uint32_t)round(exposureUs * 16 / rowTime);
  int numRows = numFractionalRows / 16;

  // exposure must not exceed ~ 1/fps.
  if (numRows < (int(max_exp_rows) - 20)) {
    success &= vi.SetRegister(
        0x3500,
        ModuleSelect::BOTH,
        uint8_t((numFractionalRows >> 16) & 0xFF),
        OV580_RUNTIME_CMD_SLEEP);
    success &= vi.SetRegister(
        0x3501,
        ModuleSelect::BOTH,
        uint8_t((numFractionalRows >> 8) & 0xFF),
        OV580_RUNTIME_CMD_SLEEP);
    success &= vi.SetRegister(
        0x3502, ModuleSelect::BOTH, uint8_t(numFractionalRows & 0xFF), OV580_RUNTIME_CMD_SLEEP);

    if (centered_exposure) {
      int cexp = 0x0ffe - numRows - 20 - (max_exp_rows - numRows) / 2;
      // centered exposure
      success &= vi.SetRegister(0x380e, ModuleSelect::BOTH, 0x0f, OV580_RUNTIME_CMD_SLEEP);
      success &= vi.SetRegister(0x380f, ModuleSelect::BOTH, 0xfe, OV580_RUNTIME_CMD_SLEEP);
      success &=
          vi.SetRegister(0x3826, ModuleSelect::BOTH, (cexp >> 8) & 0xff, OV580_RUNTIME_CMD_SLEEP);
      success &= vi.SetRegister(0x3827, ModuleSelect::BOTH, cexp & 0xff, OV580_RUNTIME_CMD_SLEEP);
    }
  } else {
    pango_print_warn(
        "Ov580:9281 exposure is too large %dus > %dus, either reduce it or reduce the fps.\n",
        int(numRows * rowTime),
        int((max_exp_rows - 20) * rowTime));
  }

  // shadow value for later use
  if (success) {
    exposure_us = numRows * rowTime;
  }
  return success;
}

bool Ov9281pair::SetGain(float gain) {
  bool success = true;
  if (gain < 1.0) {
    pango_print_warn("Ov580:9281 gain must be >= 1.\n");
  } else {
    if (gain > 15.5f) {
      pango_print_warn("Ov580:9281 Clamping gain to 15.5f\n");
      gain = 15.5f;
    }

    uint16_t fixedPointGain = uint16_t(gain * 16); // The gain register low 4 bits are fraction
    success &= vi.SetRegister(
        0x3509, ModuleSelect::BOTH, uint8_t(fixedPointGain & 0xFF), OV580_RUNTIME_CMD_SLEEP);

    // shadow value for later use
    if (success) {
      again = float(fixedPointGain) / 16;
    }
  }

  return success;
}

bool Ov9281pair::SetBlackLevel(uint16_t target) {
  bool success = true;
  if (target < 2048U) {
    success &= vi.SetRegister(
        0x4002, ModuleSelect::BOTH, uint8_t((target >> 8) & 0x07), OV580_RUNTIME_CMD_SLEEP);
    success &=
        vi.SetRegister(0x4003, ModuleSelect::BOTH, uint8_t(target & 0xFF), OV580_RUNTIME_CMD_SLEEP);
  } else {
    pango_print_warn("Ov580:9281 BlackLevelarget must be between 0 and 2048\n");
  }
  return success;
}

bool Ov9281pair::GetExposure(int& exposureUs) {
  // horizontal time
  uint8_t hi = 0;
  uint8_t mi = 0;
  uint8_t lo = 0;
  bool success = vi.GetRegister(0x3500, ModuleSelect::LEFT, hi, OV580_RUNTIME_CMD_SLEEP);
  success &= vi.GetRegister(0x3501, ModuleSelect::LEFT, mi, OV580_RUNTIME_CMD_SLEEP);
  success &= vi.GetRegister(0x3502, ModuleSelect::LEFT, lo, OV580_RUNTIME_CMD_SLEEP);

  if (success) {
    float numRows = (((hi & 0x0f) << 16) + (mi << 8) + lo) / 16.0;
    exposureUs = int(numRows * rowTime);
  }
  return success;
}

bool Ov9281pair::GetGain(float& gain) {
  uint8_t hi = 0;
  uint8_t lo = 0;
  bool success = vi.GetRegister(0x3508, ModuleSelect::LEFT, hi, OV580_RUNTIME_CMD_SLEEP);
  success &= vi.GetRegister(0x3509, ModuleSelect::LEFT, lo, OV580_RUNTIME_CMD_SLEEP);

  int fractionalGain = (((hi & 0x1f) << 8) + lo);
  gain = float(fractionalGain) / 16;

  return success;
}

bool Ov9281pair::GetBlackLevel(uint16_t& target) {
  uint8_t hi = 0;
  uint8_t lo = 0;
  bool success = vi.GetRegister(0x4002, ModuleSelect::BOTH, hi, OV580_RUNTIME_CMD_SLEEP);
  success &= vi.SetRegister(0x4003, ModuleSelect::BOTH, lo, OV580_RUNTIME_CMD_SLEEP);

  if (success) {
    target = ((hi & 0x07) << 8) + lo;
    return true;
  } else {
    return false;
  }
}

uint32_t Ov9281pair::GetMaxExposureUs() {
  return max_exp_rows * rowTime;
}

std::string Ov9281pair::SensorModelString() {
  if (sensor_model == SensorModel::OV928x)
    return "OV928x";
  else
    return "OV9782";
}
} // namespace surreal
