#include "TimerOne.h"
#include "SPI.h"

#define STATUS_LED 7

#define SERVOS 32
#define MAX_TIMINGS 36

#define ON10 PORTB |= (1<<6) // turn on pin 10
#define OFF10 PORTB &= ~(1<<6) // turn off pin 10

#define GROUPS 4
#define SERVOS_PER_GROUP 8


uint16_t group_offsets[4] = {0,251,502,753};
uint8_t group_latches[4] = {5,6,7,4};

uint8_t pin_2_num[8] = {0x08,0x04,0x02,0x01, 0x80,0x40,0x20,0x10};

uint8_t servo_positions[SERVOS];

uint16_t servo_timings[MAX_TIMINGS];
uint8_t  shift_output[MAX_TIMINGS];
uint8_t  shift_latch[MAX_TIMINGS];

uint16_t timer;
uint8_t  counter = 0;
uint8_t  pwm_active = 1;

uint8_t update_reg_flag = 0;

void setup() {
  //setup pin modes
  DDRF |= 0xF0;  // sets pins F7 to F4 as outputs
  DDRB = 0xFF;  // sets pins B0 to B7 as outputs
  pinMode(STATUS_LED,OUTPUT);
  //setup PC serial port
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(100); // delay to establish the usb connection. using the arduino while-serial command doesn't 
              // let it boot properly when there's no USB conenction.
  
  // setup SPI port
  SPI.begin(); 
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  
  //start the servo-timing timer and associated interrupt
  Timer1.initialize(10);        // initialize timer1, and set a period of 10 uS
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt  

  for(uint8_t i=0; i<MAX_TIMINGS; i++){
    servo_timings[i] = 0;
    shift_output[i] = 0xFF;
    shift_latch[i] = 0xFF;
  } 
  for(uint8_t i=0; i<SERVOS; i++){
    servo_positions[i] = 0;
  } 
  update_registers();
}

void callback(){
  if(timer == servo_timings[counter]){
    SPDR = shift_output[counter]; // push the byte to be loaded to the SPI register
    //__asm__("nop\n\tnop\n\tnop\n\t"); // pause to wait for the spi register to complete its shift out
    while(!(SPSR & (1<<SPIF))); //wait till the register completes
    PORTF &= ~(shift_latch[counter]); // clock the shift register latch pin low, setting the register
    PORTF |= shift_latch[counter];  // clock the shift register latch pin high, ready to be set low next time
    counter++;
  }
  
  timer++;
  if(timer == 1010){
    update_reg_flag=1;
  }

  if(timer == 2000){
    update_reg_flag=0;
    timer=0;
    counter=0;
  }
}


void update_registers(){
  while( update_reg_flag != 1){
    delayMicroseconds(10);
  }  
  for(uint8_t i=0; i<MAX_TIMINGS; i++){ // clear existing registers, so they can be cleanly written
    servo_timings[i] = 0;
    shift_output[i] = 0xFF;
    shift_latch[i] = 0xFF;
  } 

  uint8_t current_timing=0;
  uint8_t group=0;
  uint8_t group_active_flag=0;
  uint8_t servo_num=0;
  
  uint8_t group_on_time = 0;
  
  uint16_t servo_time=0;
  
  uint8_t group_active_servo_count = 0;
  
  // insert active servos into timing array
  for(uint8_t group=0; group<GROUPS; group++){ // go through all groups
    group_active_flag = 0;
    
    // ---- arrange the servos in the group from lowest to highest timing length -----

    // see how many active servos in the group there are
    // make a temp copy of postions to organize with
    group_active_servo_count = 0;
    uint16_t servo_positions_copy[SERVOS_PER_GROUP] = {};
    for(uint8_t servo=0; servo<SERVOS_PER_GROUP; servo++){
      if(servo_positions[SERVOS_PER_GROUP*group+servo] != 0){
        group_active_servo_count++;
      }
      servo_num = servo+SERVOS_PER_GROUP*group;
      servo_positions_copy[servo] = servo_positions[servo_num];
    }
    
    // sort them into a new order
    uint8_t group_timing_order[SERVOS_PER_GROUP] = {0xFF,0xFF,0xFF,0xFF, 0xFF,0xFF,0xFF,0xFF};
    uint8_t low_count=0;
    for(uint8_t i=0; i<group_active_servo_count; i++){
      uint8_t lowest_position=251;
      for(uint8_t j=0; j<SERVOS_PER_GROUP; j++){
        if(servo_positions_copy[j] != 0){
          if(servo_positions_copy[j] < lowest_position){
            lowest_position = servo_positions_copy[j];
            group_timing_order[low_count] = j;
          }
        }
      }
      servo_positions_copy[group_timing_order[low_count]]=0;
      low_count++;
    }
    
    
    // ---- go through the active servos and insert timings---

    // turn on all needed servos in the group at the beginning of their pulse
    if(group_active_servo_count > 0){
      group_on_time = current_timing;
      servo_timings[group_on_time] = group_offsets[group];
      shift_output[group_on_time] = 0x00; 
      shift_latch[group_on_time] = (1<<group_latches[group]);

      // turn on all the servos that will be turned on
      for(uint8_t i=0; i<group_active_servo_count; i++){
        shift_output[group_on_time] |= pin_2_num[group_timing_order[i]];
      }
      current_timing++;  
    }
    
    for(uint8_t i=0; i<group_active_servo_count; i++){ // go through all active servos in the group
      uint8_t servo = group_timing_order[i];
      servo_num = servo+SERVOS_PER_GROUP*group;
      servo_time = group_offsets[group] + servo_positions[servo_num];    // calculate the servo's time off
      shift_output[group_on_time] |= pin_2_num[servo]; // turn on the servo at the group's turn-on time

      // go over existing timings for this group, make sure there's not already one with this timing there 
      uint8_t original = 1;
      for(uint8_t j=1; j<group_active_servo_count; j++){
        if( servo_timings[j+group_on_time] == servo_time){
          shift_output[j+group_on_time] &= (~(pin_2_num[servo])); // turn off the servo at this timing
          original=0;
        }
      }    
     // create a timing if there's no existing timing
     if(original == 1){
        servo_timings[current_timing] = servo_time;
        shift_output[current_timing] = shift_output[current_timing-1] & (~(pin_2_num[servo])); // turn off the servo at this timing
        shift_latch[current_timing] = (1<<group_latches[group]);
        current_timing++;
     }
    }
    update_reg_flag=2;
  }
  
  // show the final register values
  /*
  for(uint8_t i=0; i<MAX_TIMINGS; i++){ // clear existing registers, so they can be cleanly written
    Serial.print(i);
    Serial.print(":\t");
    Serial.print(servo_timings[i]);
    Serial.print(",\t");
    Serial.print(shift_output[i],HEX);
    Serial.print(",\t");
    Serial.println(shift_latch[i],HEX);
  }
  */ 
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
  digitalWrite(STATUS_LED, HIGH);
  delay(500);
  digitalWrite(STATUS_LED, LOW);
  delay(500);
  
  for(int i=0; i<32; i++){
    changeServo(i,0);
  }
  
  changeServo(0,1500);
  changeServo(1,1500);
  changeServo(2,1500);
  changeServo(3,1500);
  
  delay(100);

  while(true){
    if(update_reg_flag == 1){
      
      update_registers();
      
    }
    else{
      if(Serial.available()) {
        char inChar = (char)Serial.read();
        switch(inChar){
          case '#':
            ON10;
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
             OFF10;
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
            Serial.println("SERVOTOR32_v1.5");
            break;
          case 'C':
            for(int i=0; i<32; i++){
              changeServo(i,1500);
            }
            Serial.println("All Centered");
            break;
          case 'K':
            for(int i=0; i<32; i++){
              changeServo(i,0);
            }
            Serial.println("All Turned Off");
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
      if(Serial1.available()){
        char inChar = (char)Serial1.read();
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
            Serial1.println("SERVOTOR32_v1.5");
            break;
          case 'C':
            for(int i=0; i<32; i++){
              changeServo(i,1500);
            }
            Serial1.println("All Centered");
            break;
          case 'K':
            for(int i=0; i<32; i++){
              changeServo(i,0);
            }
            Serial1.println("All Turned Off");
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
}

void changeServo(byte servo, short pos){
  servo_positions[servo] = pos/10;
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

