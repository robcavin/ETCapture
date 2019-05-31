#pragma once

#include <ar/core/Image/Image.h>
#include <pangolin/factory/factory_registry.h>
#include <pangolin/pangolin.h>
#include <pangolin/video/stream_info.h>
#include <pangolin/video/video.h>
#include <pangolin/video/video_interface.h>
#include <queue>

namespace surreal {
inline int checkOV580Frame(
    Eigen::Vector2f gainAndExposure,
    surreal::Image<unsigned char>& intensity,
    bool fix_last_line) {
  int errorcode = 0;
  // check meta info.
  if (!(gainAndExposure[1] > 0 && gainAndExposure[1] < 100)) {
    errorcode = errorcode | 1;
  }
  if (!(gainAndExposure[0] > 0.9 && gainAndExposure[0] < 16)) {
    errorcode = errorcode | 2;
  }

  // check that the back of the second, third and last line isn't all black.
  int tocheck[3] = {1, 2, ((int)intensity.h) - 1};
  for (int z = 0; z < 3; z++) {
    int y = tocheck[z];
    int nblack = 0;
    for (int i = 0; i < 100; i++) {
      if (intensity.Get((int)intensity.w - 1 - i, y) == 0)
        nblack++;
    }
    if (nblack > 80) {
      errorcode = errorcode | 4;
    }
  }

  // check that end of zero'th line IS all black
  {
    int nblack = 0;
    for (int i = 0; i < 100; i++) {
      if (intensity.Get((int)intensity.w - 1 - i, 0) == 0)
        nblack++;
    }
    if (nblack < 99) {
      errorcode = errorcode | 8;
    }
  }

  // replace zero'th line with first line.
  if (fix_last_line)
    memcpy(intensity.RowPtr(0), intensity.RowPtr(1), intensity.w);

  return errorcode;
}
} // namespace surreal
