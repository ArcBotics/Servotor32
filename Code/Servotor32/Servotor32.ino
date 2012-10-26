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
uint8_t pulseWidths[201][4]; //array of when to change what pins at what times
                                   //[time in 10's of uS -50uS][servo group]=group bitmask

uint8_t portBitMaskZeroes[9] = {0x01,0x02,0x04,0x08, 0x10,0x20,0x40,0x80, 0x00};
uint8_t portBitMaskOnes[9]   = {0xFE,0xFD,0xFB,0xF7, 0xEF,0xDF,0xBF,0x7F, 0xFF};

uint8_t bitMaskZeroes[9] = {0x08,0x4,0x02,0x01, 0x80,0x40,0x20,0x10, 0x00};
uint8_t bitMaskOnes[9]   = {0xF7,0xFB,0xFD,0xFE, 0x7F,0xBF,0xDF,0xEF, 0xFF};

uint8_t groupPin[4] = {5,6,7,4};

uint8_t pwm_active = 1;

void setup() {
  pinMode(STATUS_LED,OUTPUT);
  //setup pin modes
  DDRF = 0xFF;  // sets pins B0 to B7 as outputs
  pinMode(10,OUTPUT);
  //setup PC serial port
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(100); // delay to establish the usb connection. using the arduino while-serial command doesn't 
              // let it boot properly when there's no USB conenction.
  
  // setup SPI port
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


#define LARGEST_GROUP 3 //the largested-numbered group of servos. Largest possible is 
#define INCREMENT_TIME 251 //the number of 10s of microseconds between pulse groups

uint8_t group = 0;
uint16_t min_period = 50;
uint16_t max_period = INCREMENT_TIME-1;
uint16_t switch_time = INCREMENT_TIME;

uint16_t timer_inc = 0;       //number of time increments passeds

uint8_t update_flag = 0;

uint8_t output_byte = 0x00;
uint8_t output_latch_low = 0x00;
uint8_t output_latch_high = 0x00;

void callback()
{
  timer_inc++;
  // the trick here is to pre-compute as much as possible, so you're not incuring variable computation delays when shifting
  if(update_flag == 1){ //if there is an update that needs to be made
    // shift up an update to the appropriate shift register
    SPDR = output_byte; // push the byte to be loaded to the SPI register
    __asm__("nop\n\t"); // pause to wait for the spi register to complete its shift out
    //while(!(SPSR & (1<<SPIF))); //wait till the register completes
    PORTF &= output_latch_low; // clock the shift register latch pin low, setting the register
    PORTF |= output_latch_high; // clock the shift register latch pin high, ready to be set low next time
    update_flag=0;
  } 
  
  //update the servos in the appropriate group based on the groups timing array (pulseWidths)
  if((timer_inc >= min_period) && (timer_inc <= max_period)){
    if(pulseWidths[timer_inc - min_period][group] != 0xFF){ //if there are any changes to Port B
      output_byte &= pulseWidths[timer_inc-min_period][group];
      output_latch_low  = portBitMaskOnes[groupPin[group]];
      output_latch_high = portBitMaskZeroes[groupPin[group]];
      update_flag = 1;
    }
  }
  
  //move to the next servo group, bring the pins high to start that servo period
  if(timer_inc == 2000){
    timer_inc = 0;  //reset increment timer
    group = 0;
    min_period = 50;
    max_period = INCREMENT_TIME-1;
    switch_time = INCREMENT_TIME;
    pwm_active = 1; 
    output_byte = 0xFF;
    output_latch_low  = portBitMaskOnes[groupPin[group]];
    output_latch_high = portBitMaskZeroes[groupPin[group]];
    update_flag = 1;
  }
  
  if(timer_inc == switch_time){
    if(group != LARGEST_GROUP){
      group++; //increment the group
      min_period += INCREMENT_TIME; // setup the new timing parameters for the group
      max_period += INCREMENT_TIME;
      switch_time += INCREMENT_TIME;
      output_byte = 0xFF; 
      output_latch_low  = portBitMaskOnes[groupPin[group]];
      output_latch_high = portBitMaskZeroes[groupPin[group]];
      update_flag = 1; 
    }
    else{ //if its the first group of the cycle
      switch_time = 2000; // number to get a 20ms update cycle
      pwm_active = 0;
    }
  }
}

uint8_t grouServoNumTable [32] = {0,1,2,3,4,5,6,7,
                                  0,1,2,3,4,5,6,7,
                          0,1,2,3,4,5,6,7,
                          0,1,2,3,4,5,6,7};
                          
uint8_t groupTable [32] = {0,0,0,0,0,0,0,0,
                   1,1,1,1,1,1,1,1,
                   2,2,2,2,2,2,2,2,
                   3,3,3,3,3,3,3,3};
                          
void updatePWM(uint8_t servoNum, uint8_t servoPos){
  unsigned short tempPosIndex;
  unsigned short servoPosIndex;
  byte group;
  byte groupServoNum;
  
  groupServoNum = grouServoNumTable[servoNum];
  group = groupTable[servoNum];
  
  tempPosIndex = servoPositions[servoNum]; //retreive current servo position
  
  while(pwm_active == 0){ // pause while pins are updating
    delayMicroseconds(1); // should implement a double-buffer system instead
  }
  
  PORTE |= 0x40; // bring PE6 (pin 7) high, signals servo changed
  if(servoPos == 0){  //kill the servo
    pulseWidths[tempPosIndex][group] |=  bitMaskZeroes[groupServoNum]; //remove old update position   
    servoPositions[servoNum] = servoPos; //update current servo position
  }
  else{
    pulseWidths[tempPosIndex][group] |=  bitMaskZeroes[groupServoNum]; //remove old update position
    pulseWidths[servoPos-50][group] &=  bitMaskOnes[groupServoNum]; //add to new update position
    servoPositions[servoNum] = servoPos; //update current servo position
  }
  PORTE &= 0xBF; // bring PE6 (pin 7) low, signals servo changed 
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
  delay(500);
  
  for(int i=0; i<32; i++){
    changeServo(i,0);
  }
  long startTime=0;
  long stopTime=0;
  
  for(int i=0; i<32; i++){
    changeServo(i,2500);
  }

  
  while(true){
    while(Serial.available()){
      char inChar = (char)Serial.read();
      switch(inChar){
        case '#':
          startTime = micros();
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
          // !TODO: Speed this up.
          if((inServo >=0)&&(inServo <=31)&&(((inPos >= 500)&&(inPos <= 2500))||(inPos == 0))){
            changeServo(inServo,inPos);
            inServo = -1;
            inPos = -1;
          }
          numCount = 0;
          stopTime = micros();
          Serial.print("time to execute: ");
          Serial.println(stopTime-startTime);
          break;
        case 'V':
          Serial.println("SERVOTOR32_v1.4");
          break;
        case 'C':
          for(int i=0; i<32; i++){
            updatePWM(i,150);
          }
          Serial.println("All Centered");
          break;
        case 'K':
          for(int i=0; i<32; i++){
            updatePWM(i,0);
          }
          Serial.println("All Turned Off");
          break;
        case 'L':
          if(servoCounting){
            inServo = tallyCount();
            servoCounting = false;
          }
          changeServo(inServo, 0);
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

void changeServo(uint8_t servo, short pos){
  updatePWM(servo,(uint8_t)(pos/10));
}

// !TODO: increase the speed of this significantly
// Is there a tables-based approach?
// only works for numbers 3 digits or less
short tallyCount(){
   long 
   total=0;
   for(int i=0; i<numCount; i++){  
     total += powers[i]*numString[(numCount-1)-i];  
   }
   if(numCount == 0){
     total = -1;
   }
   return total;
}
