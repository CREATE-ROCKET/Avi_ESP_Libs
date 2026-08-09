#pragma once

#include <AS5047D.h>

inline void aviAs5047dPipelineApiSmoke() {
  AS5047D encoder;
  AS5047D::RawData raw{};
  AS5047D::Data data{};
  (void)encoder.startPipelinedRead();
  (void)encoder.readPipelinedRaw(raw);
  (void)encoder.readPipelined(data);
  (void)encoder.stopPipelinedRead();
  (void)encoder.pipelinedReadActive();
}
