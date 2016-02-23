#include "MjxTelemetry.h"

MjxTelemetry::MjxTelemetry(MjxModel& m): model(m), now(0), flush(false) {}

bool MjxTelemetry::ready(uint32_t flags) const
{
  if((model.config.dump_flags & flags) != flags) return false;
  if((now - model.config.dump_ts) < model.config.dump_interval) return false;
  return true;
}

void MjxTelemetry::print(double val, uint32_t flags)
{
  if(!flush) now = millis();
  if(!ready(flags)) return;
  flush = true;

  Serial.print(val, 3);
  Serial.print(F(" "));
}

void MjxTelemetry::reset()
{ 
  if(!flush) return;
  Serial.println();
  model.config.dump_ts = now;
  flush = false;
}

