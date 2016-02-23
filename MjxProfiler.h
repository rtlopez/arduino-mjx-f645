#ifndef MjxProfiler_h
#define Mjxprofiler_h

#include "Arduino.h"

typedef int64_t tm_t;

class MjxProfiler
{
  public:
    MjxProfiler(const char * name_, tm_t interval_): 
      name(name_), display_ts(0), display_int(interval_), start_ts(0), now_ts(0), sum_ts(0), count(0), min_ts(-1), max_ts(-1) {}
      
    void start()
    {
      now_ts = start_ts = micros();
    }
    
    void stop()
    {
      now_ts = micros();
      tm_t diff_ts = now_ts - start_ts;
      sum_ts += diff_ts;
      count++;
      if(min_ts == -1) min_ts = diff_ts;
      if(max_ts == -1) min_ts = diff_ts;
      min_ts = min(min_ts, diff_ts);
      max_ts = max(max_ts, diff_ts);
      dump();
    }
    
    void dump()
    {
      if(!display_int || !count || !start_ts || !now_ts) return;
      if(now_ts - display_ts > display_int * 1000)
      {
        Serial.print(F(" * PROFILER(")); Serial.print(name);
        Serial.print(F("): ")); Serial.print(min_ts / 1000.0);
        Serial.print(F(", "));  Serial.print(sum_ts / 1000.0 / count);
        Serial.print(F(", "));  Serial.print(max_ts / 1000.0);
        Serial.println();
        min_ts = -1;
        max_ts = -1;
        sum_ts = 0;
        count = 0;
        display_ts = now_ts;
      }
    }
  private:
    const char * name;
    tm_t display_ts, display_int;
    tm_t start_ts, now_ts;
    tm_t sum_ts, count;
    tm_t min_ts, max_ts;
};

#endif
