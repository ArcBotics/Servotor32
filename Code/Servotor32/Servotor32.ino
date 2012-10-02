/***************************************************
~~~~~~~~~~~~~~~~~~~~~ Servitor ~~~~~~~~~~~~~~~~~~~~~
 The Arduino-based, Open-Source 32-Servo Controller
***************************************************/
//import libraries
#include "TimerOne.h"
#include "SPI.h"

//setup pins - http://pjrc.com/teensy/td_digital.html

#define STATUS_LED 7

//setup timing variables
signed short servoPositions[32];   //array of servo position values [servo]=position in 10's of uSs
unsigned char pulseWidths[201][4]; //array of when to change what pins at what times
                                   //[time in 10's of uS -50uS][servo group]=group bitmask
unsigned short timeIncs = 0;       //number of time increments passed
unsigned char output = 0xFF;

unsigned char portBitMaskZeroes[9] = {0x01,0x02,0x04,0x08, 0x10,0x20,0x40,0x80, 0x00};
unsigned char portBitMaskOnes[9]   = {0xFE,0xFD,0xFB,0xF7, 0xEF,0xDF,0xBF,0x7F, 0xFF};

unsigned char bitMaskZeroes[9] = {0x08,0x4,0x02,0x01, 0x80,0x40,0x20,0x10, 0x00};
unsigned char bitMaskOnes[9]   = {0xF7,0xFB,0xFD,0xFE, 0x7F,0xBF,0xDF,0xEF, 0xFF};

unsigned char groupPin[4] = {5,6,7,4};

signed char pwmActive = 1;

void setup() {
  pinMode(STATUS_LED,OUTPUT);
  //setup pin modes
  DDRF = 0xFF;  // sets pins B0 to B7 as outputs
  //setup PC serial port
  Serial.begin(115200);
  while (!Serial) {};

  //setup SPI port
  SPI.begin(); 
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  
  //set initial servo positions as neutral (so they have something to change from)
  for(unsigned char i=0; i<32; i++){
    servoPositions[i]=1500;
  }
  //set inital change times (no pulse drops ever)
  for(unsigned char i=0; i<5; i++){
    for(unsigned char j=0; j<201; j++){
      pulseWidths[j][i]=0xFF;
    }
  }  
  
  //start the servo-timing timer and associated interrupt
  Timer1.initialize(10);        // initialize timer1, and set a period of 10 uS
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt  
  
}

/*
void shiftOutSpi(unsigned char outByte, unsigned char chip_latch){
  SPDR = outByte;
  while(!(SPSR & (1<<SPIF)));
  PORTF |= portBitMaskZeroes[chip_latch];
  PORTF &= portBitMaskOnes[chip_latch];
}
*/
unsigned char group = 0;
unsigned short minPeriod = 50;
unsigned short maxPeriod = 250;
unsigned short switchTime = 260;
#define INCREMENT_TIME 260 //the number of 10s of microseconds between pulse groups
#define LARGEST_GROUP 3 //the largested-numbered group of servos. Largest possible is 

byte outputBuffer = 0;
byte groupPinBuffer = 0;

byte updateFlag = 1;
byte canUpdateServos = 0;

void callback()
{
  if(updateFlag == 1){
    SPDR = outputBuffer;
    while(!(SPSR & (1<<SPIF)));
    PORTF |= portBitMaskZeroes[groupPinBuffer];
    PORTF &= portBitMaskOnes[groupPinBuffer];
    updateFlag = 0;
  }
  
  //update the servos in the appropriate group based on the groups timing array (pulseWidths)
  if((timeIncs >=minPeriod) && (timeIncs <=maxPeriod)){
    if(pulseWidths[timeIncs-minPeriod][group] != 0xFF){ //if there are any changes to Port B
      output &= pulseWidths[timeIncs-minPeriod][group];
      outputBuffer = output;
      groupPinBuffer = groupPin[group];
      updateFlag = 1;
    }
  }
  
  //move to the next servo group, bring the pins high to start that servo period
  if(timeIncs==switchTime){
    if(group==LARGEST_GROUP){ //if its the first group of the cycle
      if(switchTime == 2000){ // number to get a 20ms update cycle
        canUpdateServos = 0;
        timeIncs=0;  //reset increment timer
        group=0;
        minPeriod=50;
        maxPeriod=250;
        switchTime=260;
        pwmActive=1; 
        output = 0xFF;
        outputBuffer = output;
        groupPinBuffer = groupPin[group];
        updateFlag = 1;
      }
      else{
        canUpdateServos = 1;
        switchTime=2000; // number to get a 20ms update cycle
        pwmActive=0;
      }
    }
    else{
      group++; //increment the group
      minPeriod += INCREMENT_TIME;
      maxPeriod += INCREMENT_TIME;
      switchTime += INCREMENT_TIME;
      output = 0xFF;
      outputBuffer = output;
      groupPinBuffer = groupPin[group];
      updateFlag = 1; 
    }
  }
  timeIncs++;
}

void updatePWM(byte servoNum, short servoPos){
  
  servoPos = servoPos;
  unsigned short tempPosIndex;
  unsigned short servoPosIndex;
  byte group;
  byte groupServoNum;
  
  if((servoNum >=0)&&(servoNum <=7)){
    groupServoNum = servoNum-0;
    group = 0;
  }
  else if((servoNum >=8)&&(servoNum <=15)){
    groupServoNum = servoNum-8;
    group = 1;
  }
  else if((servoNum >=16)&&(servoNum <=23)){
    groupServoNum = servoNum-16;
    group = 2;
  }
  else if((servoNum >=24)&&(servoNum <=31)){
    groupServoNum = servoNum-24;
    group = 3;
  }
  else return;
  
  tempPosIndex = servoPositions[servoNum]/10-50; //retreive current servo position
  servoPosIndex = servoPos/10-50;

  while(canUpdateServos == 0){ // pause while pins are updating
    delayMicroseconds(10);
  }
  
  PORTC |= 0x80; // bring PC7 (pin 13) high 
  if(servoPos == -1){  //kill the servo
    pulseWidths[tempPosIndex][group] |=  bitMaskZeroes[groupServoNum]; //remove old update position   
    servoPositions[servoNum] = servoPos; //update current servo position
  }
  else{
    pulseWidths[tempPosIndex][group] |=  bitMaskZeroes[groupServoNum]; //remove old update position
    pulseWidths[servoPosIndex][group] &=  bitMaskOnes[groupServoNum]; //add to new update position
    servoPositions[servoNum] = servoPos; //update current servo position
  }
  PORTC &= 0x7F; // bring PC7 (pin 13) low 
  
}

boolean debug = false;
boolean testMode = false;
boolean servoCounting = false;
boolean posCounting = false;

byte numString[6];
int powers[] = {1,10,100,1000};


byte numCount = 0;
unsigned short total = 0;
short inServo = -1;
short inPos = -1;
void loop() {
  // Status LED light to indicate its turned on
  digitalWrite(STATUS_LED, HIGH);
  delay(500);
  digitalWrite(STATUS_LED, LOW);
  
  while(true){
    while (Serial.available()) {
      char inChar = (char)Serial.read();
      switch(inChar){
        case '#':
          servoCounting = true;
          numCount = 0;
          inServo = -1;
          inPos = -1;
          break;
        case 'P':
          if(servoCounting){
            inServo = tallyCount();
            servoCounting = false;
          }
          posCounting =  true;
          numCount = 0;
          break; 
        case '\r':
        case '\n':
          if(posCounting){
            inPos = tallyCount();
            posCounting = false;
          }
          if((inServo >=0)&&(inServo <=31)&&(((inPos >= 500)&&(inPos <= 2500))||(inPos == -1))){
            changeServo(inServo,inPos);
            inServo = -1;
            inPos = -1;
          }
          numCount = 0;
          break;
        case 'V':
          Serial.println("SERVOTOR32_v1\n");
          break;
        case 'L':
          if(servoCounting){
            inServo = tallyCount();
            servoCounting = false;
          }
          changeServo(inServo, -1);
          break;
        default:
          if((inChar > 47)&&(inChar < 58)){
            if(numCount<4){
              numString[numCount] = inChar-48;
              numCount++;
            }
          }
          break;
      } 
    }
  }
}

void changeServo(byte servo, short pos){
  updatePWM(servo,pos);
}

short tallyCount(){
   total=0;
   for(int i=0; i<numCount; i++){  
     total += powers[i]*numString[(numCount-1)-i];  
   }
   if(numCount == 0){
     total = -1;
   }
   return total;
}
