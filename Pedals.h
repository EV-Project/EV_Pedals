#ifndef PEDALS_H 
#define PEDALS_H


#include <inttypes.h>
#include <Arduino.h> // Arduino 1.0
#include <FlexCAN.h>
#include <CANcallbacks.h>

const uint8_t reverseSwBit = 0;
const uint8_t frontSDBit = 1;
const uint8_t rearSDBit = 2;
const uint8_t estopBit = 3;
const uint8_t brakeWarnBit = 4;
const uint8_t throttleWarnBit = 5;



class Pedals{
  public:
    Pedals();
    //~Pedals();
    float getThrottle();
    float getBrake();
    bool getReverse();
    bool getSDfront();
    bool getSDrear();
    bool getEstop();
    bool processMessage(CAN_message_t &message);
  private:
    float brakeVal;
    float throttleVal;
  
    bool brakesOK;
    bool throttleOK;
    bool reverse;
    bool SDfront;
    bool SDrear;
    bool Estop;
};




#endif
