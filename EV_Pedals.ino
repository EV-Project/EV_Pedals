#//include <KellyCAN.h>
#include <FlexCAN.h>
#include <CANcallbacks.h>
#include <ChallengerEV.h>

const int revPins[] = {14, 8, 16};
const int keyPins[] = {14, 9, 16};
const int estopPins[] = {14, 7, 16};


const int frontSdPin = 9;
const int rearSdPin = 10;

const int revLampPin = 24;

bool EstopMode = false;
bool reversingMode = false;
bool revBounce = false;
bool keyState = false;


bool SDfront = false;
bool SDrear = false;

//bool Estop = false;



FlexCAN CANbus(1000000);

//static CAN_message_t msg,rxmsg;
//CanBus canbus;

CANcallbacks canbus(&CANbus);
//KellyCAN motor(&canbus, 107, 115);

const int throttlePin = 20;
const int brakePins[6] = {14, 15, 16, 14, 17, 16};

float brakeMax = 0.9;
float brakeMin = 0.1;
float brakeTolerance = 0.05;

float readPot(const int *pins){
//read one way
	digitalWrite(pins[0], HIGH);
	digitalWrite(pins[2], LOW);
  delay(1);
	float Pval = (float)analogRead(pins[1])/1024.0;
  //Serial.print("Pval: ");
  //Serial.println(Pval);
  //read the other way
	digitalWrite(pins[0], LOW);
	digitalWrite(pins[2], HIGH);
  delay(1);
	float Nval = (float)analogRead(pins[1])/1024.0;
  //Serial.print("Nval: ");
  //Serial.println(Nval);
  
	if(Pval<brakeMin) return -1;
	if(Pval>brakeMax) return -1;
	if(Nval<brakeMin) return -1;
	if(Nval>brakeMax) return -1;
	if(abs(Pval + Nval - 1.0)>brakeTolerance) return -1;
	return (((Pval+(1.0-Nval))/2.0) - brakeMin)*(1/(brakeMax-brakeMin));
}

bool readSwitch(const int *pins, bool &buttonState){
  digitalWrite(pins[0], LOW);
  digitalWrite(pins[2], HIGH);
    delay(1);
  buttonState = digitalRead(pins[1]);
  digitalWrite(pins[0], HIGH);
  digitalWrite(pins[2], LOW);
    delay(1);
  bool buttonCheck = digitalRead(pins[1]);

  if(buttonState == buttonCheck){
    return false;
  }else{
    return true;
  }
}


bool readThrottle(float &throttleVal){
  int raw = analogRead(throttlePin)-100;
  if(raw<100) raw = 0;
  throttleVal = (float)raw/(float)(1024-100);
  return true;
}

/**
 * Read both pots on the brake pedal, output in the reference brakeVal
 * Returns: true if the pots are working fine, false if only one pot is OK.
 * if both pots check ok but still don't agree, return pot A.
**/
bool readBrakes(float &brakeVal){
	float potA = readPot(brakePins);
	float potB = readPot(&brakePins[3]);
  if((potA == -1)&&(potB == -1)){
    brakeVal =  -1;
    return false;
  }else if(potA == -1){
    brakeVal = potB;
    return false;
  }else if(potB == -1){
    brakeVal = potA;
    return false;
  }else{
    if(abs(potA-potB)<brakeTolerance){
      brakeVal = (potA+potB)/2;
      return true;
    }else{
      brakeVal = potA;
      return false;
    }
  }
}

void setEstop(){
  EstopMode = true;
}
void clearEstop(){
  EstopMode = false;
}



void setup() {
  Serial.begin(9600);

  Serial.println("Pedals for the EV");

  CAN_filter_t pedalFilter;
  pedalFilter.rtr = 0;
  pedalFilter.ext = 0;
  pedalFilter.id = 160;
  CANbus.begin(pedalFilter);
  
  //canbus.set_callback(115, &processMessage);
  pinMode(brakePins[0], OUTPUT);
  pinMode(brakePins[1], INPUT);
  pinMode(brakePins[2], OUTPUT);
  pinMode(brakePins[3], OUTPUT);
  pinMode(brakePins[4], INPUT);
  pinMode(brakePins[5], OUTPUT);
  
  pinMode(throttlePin, INPUT);
  
  pinMode(revPins[1], INPUT);
  pinMode(keyPins[1], INPUT);
  pinMode(estopPins[1], INPUT);
}

/*
typedef struct CAN_message_t {
  uint32_t id; // can identifier
  uint8_t ext; // identifier is extended
  uint8_t len; // length of data
  uint16_t timeout; // milliseconds, zero will disable waiting
  uint8_t buf[8];
} CAN_message_t;
*/

void loop() {
  // put your main code here, to run repeatedly:

  float brakeVal;
  bool brakesOK = readBrakes(brakeVal);
  //Serial.print("Brakes: ");
  //Serial.println(brakeVal);
  
  float throttleVal;
  bool throttleOK = readThrottle(throttleVal);
  Serial.print("Throttle: ");
  Serial.println(throttleVal);
  Serial.print("Brakes: ");
  Serial.print(brakeVal);
  if(brakesOK){
    Serial.println(" OK");
  }else {
    Serial.println(" Bad");
  }
  if(reversingMode) Serial.println("Reverse");
  if(EstopMode) Serial.println("E-Stop");
  if(!keyState) Serial.println("Powered Down");

  bool revSwState = false;
  bool keySwState = false;
  bool estopSwState = false;
  if(!readSwitch(revPins, revSwState)) Serial.println("Switch Error  ******************************");
  if(!readSwitch(keyPins, keySwState)) Serial.println("Switch Error  ******************************");
  if(!readSwitch(estopPins, estopSwState)) Serial.println("Switch Error  ******************************");


  if(revSwState){   //one-shot 
    if(revBounce == false){
      reversingMode = !reversingMode;
      revBounce = true;
    }
  }else{  //pin is high
    revBounce = false;
  }

//  SDfront = (digitalRead(frontSdPin) == LOW);
//  SDrear = (digitalRead(rearSdPin) == LOW);

  if(estopSwState){   //latch estop until key reset
    setEstop();
  }
  
  if(keyState != keySwState){
    if(keySwState){   //Power Up
      keyState = keySwState;
      SDfront = false;
      SDrear = false;
    }else{  //Power Down
      keyState = keySwState;
      reversingMode = false;
      SDfront = true;
      SDrear = true;
      clearEstop();
    }
  }
  



  uint8_t switchesVal = 0;
  if(reversingMode) switchesVal |= 1 << reverseSwBit;
  if(SDfront) switchesVal |= 1 << frontSDBit;
  if(SDrear) switchesVal |= 1 << rearSDBit;
  if(EstopMode) switchesVal |= 1 << estopBit;
  if(!brakesOK) switchesVal |= 1 << brakeWarnBit;
  if(!throttleOK) switchesVal |= 1 << throttleWarnBit;

  //for(int i=2; i<nWheels; i++){
  for(int i=0; i<nWheels; i++){
    //load a blank message
    CAN_message_t message = {wheel[i].managerID,0,3, 0, 0,0,0,0,0,0,0,0};

    //don't transmit any throttle to any wheels in shutdown.
    if(wheel[i].isFront){
      if(SDfront || EstopMode){
        message.buf[0] = 0;
      }else{
        message.buf[0] = (uint8_t)(255*throttleVal);
      }
    }else{
      if(SDrear || EstopMode){
        message.buf[0] = 0;
      }else{
        message.buf[0] = (uint8_t)(255*throttleVal);
      }
    }
    
    message.buf[1] = (uint8_t)(255*brakeVal);
    message.buf[2] = switchesVal;
    canbus.transmit(message);
  }

  //Serial.println("sent");

  delay(20); //replace with a real frequency
}
