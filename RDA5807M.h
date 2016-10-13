#ifndef RDA5807M_h
#define RDA5807M_h

#if ARDUINO >= 100
 #include "Arduino.h"
 #define WIRE_WRITE Wire.write
#else
 #include "WProgram.h"
 #define WIRE_WRITE Wire.send
#endif

class RDA5807M {
  public:
    void begin();
}
