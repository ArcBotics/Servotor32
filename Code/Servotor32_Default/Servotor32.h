#ifndef Servotor32_h
#define Servotor32_h

#include "Servotor32_SPI.h"
#include "Servotor32_TimerOne.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "Arduino.h"

#define STATUS_LED 7

#define SERVOS 32
#define MAX_TIMINGS 36

#define GROUPS 4
#define SERVOS_PER_GROUP 8


class Servotor32 {
public:
  Servotor32();
  void begin();
  
  long unsigned int micros_new();

  long unsigned int millis_new();
  void delay_ms(long unsigned int);
  void delay_us(long unsigned int);
  
  void changeServo(byte, short);
  
  void printStatus();
  
  void processChar(char);
  
  float ping();
  float multiPing(unsigned short);

private:
  void update_registers_fast(byte, signed short);
  void delete_from_sorted_array(byte, byte, signed short);
  void add_to_sorted_array(byte, byte, signed short);
  static void callback();
  short tallyCount();

};

#endif

