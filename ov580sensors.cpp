#include <ov580sensors.h>
#include <ov580.h>
#include <ov7251pair.h>
#include <ov9281pair.h>

#include <memory>

namespace surreal {
std::unique_ptr<Ov580Sensors> Ov580Sensors::Create(
    Ov580Sensors::SensorModel type,
    Ov580Video& ov580interface,
    bool centered_exposure) {
  switch (type) {
    case Ov580Sensors::SensorModel::OV928x:
    case Ov580Sensors::SensorModel::OV9782:
      return std::unique_ptr<Ov9281pair>(new Ov9281pair(
          ov580interface, centered_exposure, type)); // we don't have make_unique on fb centos
      break;
    default:
      return std::unique_ptr<Ov7251pair>(new Ov7251pair(ov580interface, centered_exposure, type));
      break;
  }
}

void Ov580Sensors::DumpRegsToFile(std::string filename) {
  FILE* fp = fopen(filename.c_str(), "w");
  fprintf(fp, "register\tvalue\n");

  uint8_t val;
  uint16_t intervals[3][2] = {{0x0100, 0x01FF}, {0x0300, 0x03FF}, {0x3000, 0x5FFF}};
  bool success = true;
  for (size_t r = 0; r < (sizeof(intervals) / sizeof(intervals[0])); r++) {
    for (int idx = intervals[r][0]; idx <= intervals[r][1]; idx++) {
      success &= vi.GetRegister(idx, Ov580Sensors::ModuleSelect::LEFT, val);
      fprintf(fp, "0x%04x\t0x%02x\n", idx, val);
      if (!success) {
        printf("issues reading from sensor at register %04x\n", idx);
        break;
      }
    }
    if (!success) {
      break;
    }
  }

  fclose(fp);
}
} // namespace surreal
