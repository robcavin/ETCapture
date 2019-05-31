#include <ov7251pair.h>
#include <ov580.h>
#include <iomanip>
#include <thread>

namespace surreal {
Ov7251pair::Ov7251pair(
    Ov580Video& ov580interface,
    bool centered_exp,
    SensorModel type,
    bool testmode)
    : Ov580Sensors(ov580interface, centered_exp, type, testmode) {
  if (type == surreal::Ov580Sensors::SensorModel::OV7251)
    sccb_addr = SCCBADDR_7251;
  else
    sccb_addr = SCCBADDR_7750;
}

Ov7251pair::~Ov7251pair() {}

bool Ov7251pair::Init() {
  uint8_t hi = 0;
  uint8_t lo = 0;

  if (!CheckSensorId()) {
    if (!test_mode) {
      pango_print_warn("Ov580:7251 Unexpected sensor ID, stop setting up any registers!");
      return false;
    } else {
      pango_print_warn("Test mode: Ov580:7251 Unexpected sensor ID Will try to keep going...");
    }
  }

  bool success = vi.GetRegister(0x380c, ModuleSelect::LEFT, hi);
  success &= vi.GetRegister(0x380d, ModuleSelect::LEFT, lo);
  hts = hi << 8 | lo;
  rowTime = double(hts) / SCLKFreqMHZ;

  // enable embedded line as first line of frame
  // 9th byte contains numRows of exposure
  // 4th, 6th, 7th byte hi 4 bits contain gain
  if (USE_EMBEDDED_LINE) {
    embedded_line = vi.SetRegister(0x4307, ModuleSelect::BOTH, 0x31);
  }
  return success;
}

bool Ov7251pair::StopStreaming() {
  return vi.SetRegister(SC_MODE_SELECT, ModuleSelect::BOTH, SC_MODE_SELECT_STDBY);
}

bool Ov7251pair::StartStreaming() {
  return vi.SetRegister(SC_MODE_SELECT, ModuleSelect::BOTH, SC_MODE_SELECT_STREAM);
}

bool Ov7251pair::EnableStrobe(uint32_t strobe_duration_us) {
  // for now we assume stobe is setup only once and never changes at runtime.
  bool success = true;

  // Set the strobe timing step
  constexpr uint16_t stepSizeInUs = 1;
  uint16_t const stepSizeInSysClk = (uint16_t)round(stepSizeInUs * SCLKFreqMHZ);
  success &= vi.SetRegister(0x3b94, ModuleSelect::BOTH, uint8_t((stepSizeInSysClk >> 8) & 0xFF));
  success &= vi.SetRegister(0x3b95, ModuleSelect::BOTH, uint8_t(stepSizeInSysClk & 0xFF));

  for (auto side : {ModuleSelect::LEFT, ModuleSelect::RIGHT}) {
    // Enable manual control of strobe timing step
    success &= vi.SetRegister(0x3b96, side, 0x08);

    // Set Strobe pin to output
    success &= vi.SetRegister(0x3005, side, 0x08);

    if (strobe_duration_us == 0) {
      // Force strobe output value to low
      success &= vi.SetRegister(0x3009, side, 0x00);
      // Make sure strobe is not active for any frames
      success &= vi.SetRegister(0x3b81, ModuleSelect::BOTH, 0x00);
    } else {
      // Make sure strobe is active for all frames
      success &= vi.SetRegister(0x3b81, ModuleSelect::BOTH, 0xFF);
    }
  }

  // Compute strobe duration register value
  uint32_t strobeDurationInSteps = strobe_duration_us / stepSizeInUs;

  // Set strobe delay to zero.
  success &= vi.SetRegister(0x3b88, ModuleSelect::BOTH, 0x00);
  success &= vi.SetRegister(0x3b89, ModuleSelect::BOTH, 0x00);
  success &= vi.SetRegister(0x3b8a, ModuleSelect::BOTH, 0x00);
  success &= vi.SetRegister(0x3b8b, ModuleSelect::BOTH, 0x00);

  // Set strobe duration
  success &= vi.SetRegister(0x3b8c, ModuleSelect::BOTH, (strobeDurationInSteps >> 24) & 0xFF);
  success &= vi.SetRegister(0x3b8d, ModuleSelect::BOTH, (strobeDurationInSteps >> 16) & 0xFF);
  success &= vi.SetRegister(0x3b8e, ModuleSelect::BOTH, (strobeDurationInSteps >> 8) & 0xFF);
  success &= vi.SetRegister(0x3b8f, ModuleSelect::BOTH, strobeDurationInSteps & 0xFF);

  return success;
}

bool Ov7251pair::GetStrobe(uint32_t& strobe_duration_us) {
  // Get strobe duration
  constexpr uint16_t stepSizeInUs = 1;
  bool success = true;

  union {
    uint8_t result[sizeof(uint32_t)];
    uint32_t value;
  } converter;

  success &=
      vi.GetRegister(0x3b8c, ModuleSelect::LEFT, converter.result[3], OV580_RUNTIME_CMD_SLEEP);
  success &=
      vi.GetRegister(0x3b8d, ModuleSelect::LEFT, converter.result[2], OV580_RUNTIME_CMD_SLEEP);
  success &=
      vi.GetRegister(0x3b8e, ModuleSelect::LEFT, converter.result[1], OV580_RUNTIME_CMD_SLEEP);
  success &=
      vi.GetRegister(0x3b8f, ModuleSelect::LEFT, converter.result[0], OV580_RUNTIME_CMD_SLEEP);

  strobe_duration_us = converter.value * stepSizeInUs;
  return success;
}

bool Ov7251pair::EnableExternalTrigger() {
  bool success = true;
  // enable FSIN
  success &= vi.SetRegister(ANA_CORE_6, ModuleSelect::BOTH, 0x00);
  success &= vi.SetRegister(TIMING_REG_23, ModuleSelect::BOTH, 0x30);

  return success;
}

bool Ov7251pair::CheckSensorId() {
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

bool Ov7251pair::GetSerials(std::string& sn_r, std::string& sn_l) {
  bool r = GetSensorSerial(ModuleSelect::RIGHT, sn_r) && GetSensorSerial(ModuleSelect::LEFT, sn_l);
  return r;
}

bool Ov7251pair::SetPeriod(uint32_t period_us) {
  bool success = true;
  max_exp_rows = uint32_t((float(period_us) / rowTime) - 20);
  pango_print_info("Ov580:7251 max exposure %dus\n", int(max_exp_rows * rowTime));

  // note that centered_exposure -> external_trigger.
  if (!centered_exposure) {
    // horizontal time
    int vts = (SCLKFreqMHZ * period_us) / (hts);

    success &= vi.SetRegister(0x380e, ModuleSelect::BOTH, (vts >> 8) & 0xFF);
    success &= vi.SetRegister(0x380f, ModuleSelect::BOTH, vts & 0xFF);
    if (!success)
      pango_print_warn("Ov580:7251 not able to set period.\n");
  }

  return success;
}

bool Ov7251pair::ReadOTP(ModuleSelect sensor, std::vector<uint8_t>& bytes) {
  bool success = true;
  // configure PLL
  success &= vi.SetRegister(OTP_PLL, sensor, OTP_PLL_ON);
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  success &= vi.SetRegister(0x3018, sensor, 0x10);
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

bool Ov7251pair::GetSensorSerial(ModuleSelect sensor, std::string& s) {
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

    // Serial number is only the first 16 bytes...
    bytes.resize(16);
    cnt--;
    too_many_zeros = false;
    consecutive_zeros_cnt = 0;
    for (const auto& byte : bytes) {
      consecutive_zeros_cnt = (byte == 0) ? consecutive_zeros_cnt + 1 : 0;
      if (consecutive_zeros_cnt > MAX_CONSECUTIVE_ZEROS_IN_SERIAL)
        too_many_zeros = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  } while ((too_many_zeros || ((bytes.back() & 0x03) == 0)) && (cnt > 0));

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

bool Ov7251pair::DisableAAA() {
  // disable AWB
  bool success = vi.SetRegister(0x3400, ModuleSelect::BOTH, 0x04);
  success &= vi.SetRegister(0x3401, ModuleSelect::BOTH, 0x00);
  success &= vi.SetRegister(0x3402, ModuleSelect::BOTH, 0x04);
  success &= vi.SetRegister(0x3403, ModuleSelect::BOTH, 0x00);
  success &= vi.SetRegister(0x3404, ModuleSelect::BOTH, 0x04);
  success &= vi.SetRegister(0x3405, ModuleSelect::BOTH, 0x00);
  success &= vi.SetRegister(0x3406, ModuleSelect::BOTH, 0x01);

  // manual gain
  success &= vi.SetRegister(0x3509, ModuleSelect::BOTH, 0x12);

  return success;
}

void Ov7251pair::GetFrameProperties(
    double& againl,
    double& againr,
    int& expl_us,
    int& expr_us,
    unsigned char* image,
    size_t offset0,
    size_t offset1) {
  if (embedded_line) {
    unsigned char* row = image + offset0;
    expl_us = ((int(row[EL_EXPTIMEH_ID]) << 8) + int(row[EL_EXPTIMEL_ID])) * rowTime;
    const float againl_r = int(row[EL_ANALOGGAIN_ID_R]) / 16.0;
    const float againl_g = int(row[EL_ANALOGGAIN_ID_G]) / 16.0;
    const float againl_b = int(row[EL_ANALOGGAIN_ID_B]) / 16.0;
    againl = againl_r;
    // update pointer to parse embedded line of second image.
    row = image + offset1;
    expr_us = ((int(row[EL_EXPTIMEH_ID]) << 8) + int(row[EL_EXPTIMEL_ID])) * rowTime;
    const float againr_r = int(row[EL_ANALOGGAIN_ID_R]) / 16.0;
    const float againr_g = int(row[EL_ANALOGGAIN_ID_G]) / 16.0;
    const float againr_b = int(row[EL_ANALOGGAIN_ID_B]) / 16.0;
    againr = againr_r;

    if (againl_r < 0.5 || againl_g < 0.5 || againl_b < 0.5 || againr_r < 0.5 || againr_g < 0.5 ||
        againr_b < 0.5 || againl_r != againl_g || againl_r != againl_b || againr_r != againr_g ||
        againr_r != againr_b || expl_us < 1 || expr_us < 1) {
      pango_print_warn(
          "Ov580:7251 grab error: Corrupted Frame! Gains: (%.2f / %.2f / %.2f) / (%.2f / %.2f / %.2f); "
          "Exposures "
          "%d / "
          "%d\n",
          againl_r,
          againl_g,
          againl_b,
          againr_r,
          againr_g,
          againr_b,
          expl_us,
          expr_us);
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

bool Ov7251pair::SetExposure(int exposureUs) {
  bool success = true;
  if (exposureUs < rowTime)
    exposureUs = rowTime;
  // horizontal time
  int numRows = (int)round(exposureUs / rowTime);

  // exposure must not exceed ~ 1/fps.
  if (numRows < (int(max_exp_rows) - 20)) {
    success &=
        vi.SetRegister(0x3500, ModuleSelect::BOTH, (numRows >> 12) & 0xFF, OV580_RUNTIME_CMD_SLEEP);
    success &=
        vi.SetRegister(0x3501, ModuleSelect::BOTH, (numRows >> 4) & 0xFF, OV580_RUNTIME_CMD_SLEEP);
    success &=
        vi.SetRegister(0x3502, ModuleSelect::BOTH, (numRows << 4) & 0xFF, OV580_RUNTIME_CMD_SLEEP);

    if (centered_exposure) {
      int cexp = 0xfffe - numRows - 20 - (max_exp_rows - numRows) / 2;
      // centered exposure
      success &= vi.SetRegister(0x380e, ModuleSelect::BOTH, 0xff, OV580_RUNTIME_CMD_SLEEP);
      success &= vi.SetRegister(0x380f, ModuleSelect::BOTH, 0xfe, OV580_RUNTIME_CMD_SLEEP);
      success &=
          vi.SetRegister(0x3826, ModuleSelect::BOTH, (cexp >> 8) & 0xff, OV580_RUNTIME_CMD_SLEEP);
      success &= vi.SetRegister(0x3827, ModuleSelect::BOTH, cexp & 0xff, OV580_RUNTIME_CMD_SLEEP);
    }
  } else {
    pango_print_warn(
        "Ov580:7251 exposure is too large %d > %d, either reduce it or reduce the fps.\n",
        numRows,
        max_exp_rows);
  }

  // shadow value for later use
  if (success) {
    exposure_us = numRows * rowTime;
  }
  return success;
}

bool Ov7251pair::SetGain(float gain) {
  bool success = true;
  if (gain < 1.0) {
    pango_print_warn("Ov580:7251 gain must be >= 1.\n");
  } else {
    if (gain > 15.5f) {
      pango_print_warn("Ov580:7251 Clamping gain to 15.5f\n");
      gain = 15.5f;
    }

    uint8_t fixedPointGain = (uint8_t)(gain * 0x10); // The gain register low 4 bits are fraction
    success &= vi.SetRegister(0x350b, ModuleSelect::BOTH, fixedPointGain, OV580_RUNTIME_CMD_SLEEP);

    // shadow value for later use
    if (success) {
      again = float(fixedPointGain) / 0x10;
    }
  }

  return success;
}

bool Ov7251pair::SetBlackLevel(uint16_t target) {
  bool success = true;
  if (target < 255U) {
    success &= vi.SetRegister(0x4009, ModuleSelect::BOTH, uint8_t(target), OV580_RUNTIME_CMD_SLEEP);
  } else {
    pango_print_warn("Ov580:7251 BlackLevelarget must be between 0 and 255\n");
  }
  return success;
}

bool Ov7251pair::GetExposure(int& exposureUs) {
  // horizontal time
  uint8_t hi, mi, lo;
  bool success = vi.GetRegister(0x3500, ModuleSelect::LEFT, hi, OV580_RUNTIME_CMD_SLEEP);
  success &= vi.GetRegister(0x3501, ModuleSelect::LEFT, mi, OV580_RUNTIME_CMD_SLEEP);
  success &= vi.GetRegister(0x3502, ModuleSelect::LEFT, lo, OV580_RUNTIME_CMD_SLEEP);

  if (success) {
    int numRows = ((hi & 0x0f) << 12) + ((mi & 0xff) << 4) + ((lo & 0xff) >> 4);
    exposureUs = numRows * rowTime;
  }
  return success;
}

bool Ov7251pair::GetGain(float& gain) {
  uint8_t hi;
  if (vi.GetRegister(0x350b, ModuleSelect::LEFT, hi, OV580_RUNTIME_CMD_SLEEP)) {
    gain = float(hi) / 0x10;
    return true;
  } else {
    return false;
  }
}

bool Ov7251pair::GetBlackLevel(uint16_t& target) {
  uint8_t target8;
  bool success = vi.GetRegister(0x4009, ModuleSelect::LEFT, target8, OV580_RUNTIME_CMD_SLEEP);
  if (success) {
    target = target8;
    return true;
  } else {
    return false;
  }
}

uint32_t Ov7251pair::GetMaxExposureUs() {
  return max_exp_rows * rowTime;
}

std::string Ov7251pair::SensorModelString() {
  if (sensor_model == SensorModel::OV7251)
    return "OV7251";
  else
    return "OV7750";
}
} // namespace surreal
