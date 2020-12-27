// data_struct.h
#ifndef data_struct_h
#define data_struct_h

#include "Arduino.h" //needed to include standard arduino library
 struct data_struct {
  int height;
  int temperature;
  int acceleration[3];
  int orientation[3];
  
  int reset(void);
} Data ;
 
#endif
