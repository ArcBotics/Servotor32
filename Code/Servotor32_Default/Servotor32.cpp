#include "Servotor32.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "Arduino.h"

#include "Servotor32_SPI.h"
#include "Servotor32_TimerOne.h"

Servotor32::Servotor32()
{  

}

//stores information about the servos and groups
signed short servo_positions[SERVOS]; // where the servos are currently (supposed to be) at
signed char servos_sorted[GROUPS][SERVOS_PER_GROUP]; // index in servo_timings to where the servo ends
signed char servos_active_in_group[GROUPS]; // the number of servos in a group currently active
uint8_t active_servos_hex[GROUPS];

// all updates to shift registers in order of their updates
signed short servo_timings[MAX_TIMINGS]; // the timing where the change occurs
uint8_t  shift_output[MAX_TIMINGS];  // the output of the shift register
uint8_t  shift_latch[MAX_TIMINGS];   // the shift register latch used

// keeps track of whether its safe or not to update the servos
uint8_t update_reg_flag = 0;

// variables for the callback
uint16_t timer;
uint8_t  counter = 0;
uint8_t  pwm_active = 1;

uint16_t group_offsets[4] = {0,251,502,753};
uint8_t group_latches[4] = {5,6,7,4};
uint8_t pin_2_num[8] = {0x08,0x04,0x02,0x01, 0x80,0x40,0x20,0x10};

void Servotor32::begin(){
  //setup pin modes
  DDRF |= 0xF0;  // sets pins F7 to F4 as outputs
  DDRB = 0xFF;  // sets pins B0 to B7 as outputs
  
  //setup PC serial port
  Serial.begin(9600);
  // reconfigure bluetooth module to 9600 baud id needed
  Serial1.begin(115200);     // Changed from 9600 baud
  Serial1.print("AT+BAUD4"); // Tell the module to change the baud rate to 9600
  delay(1100); // Wait a notch over 1 second to make sure the setting "sticks"
  Serial1.begin(9600);     // Changed from 9600 baud
  
  SPI.begin(); 
  SPI.setClockDivider(SPI_CLOCK_DIV2); 

  Timer1.initialize(10);
  Timer1.attachInterrupt(callback);

  for(byte i=0; i<SERVOS; i++){
    servo_positions[i] = -1;
  }
  for(byte i=0; i<GROUPS; i++){
    for(byte j=0; j<SERVOS_PER_GROUP; j++){
      servos_sorted[i][j] = -1;
    }
  }
  
  for(uint8_t i=0; i<MAX_TIMINGS; i++){
    servo_timings[i] = 0;
    shift_output[i] = 0xFF;
    shift_latch[i] = 0xFF;
  } 

  TIMSK0 &= ~(_BV(TOIE0)); // disables the arduino delay function, but also
                           // all but eliminates servo jitter 
  TIMSK2 &= ~(_BV(TOIE2)); // disable the arduino tone  function, but also
                           // also helps eliminate some jitter
  TIMSK3 &= ~(_BV(TOIE3)); // for good measure
  TIMSK4 &= ~(_BV(TOIE4)); // for good measure 
}

long unsigned int us_counter = 0;
long unsigned int startTime = 0; 
long unsigned int currentTime = 0; 
long unsigned int last_update = 0;

long unsigned int Servotor32::micros_new(){
  return us_counter;
}

long unsigned int Servotor32::millis_new(){
  return us_counter/1000;
}

void Servotor32::delay_ms(long unsigned int delay_time){
  startTime = millis_new();
  currentTime = millis_new() - startTime;
  while(currentTime < delay_time){
    delayMicroseconds(10);
    currentTime = millis_new() - startTime;
  }
}

void Servotor32::delay_us(long unsigned int delay_time){
  startTime = micros_new();
  currentTime = micros_new() - startTime;
  while(currentTime < delay_time){
    delayMicroseconds(10);
    currentTime = micros_new() - startTime;
  }
}

void Servotor32::callback(){
  cli();
  if(timer < 1100){ // keep it from updating servos mid-array change by some weird coincidence
    if(timer == servo_timings[counter]){ // if the time has arrived to update a shift reg
      SPDR = shift_output[counter]; // push the byte to be loaded to the SPI register
      while(!(SPSR & (1<<SPIF))); //wait till the register completes
      PORTF &= ~(shift_latch[counter]); // clock the shift register latch pin low, setting the register
      PORTF |= shift_latch[counter];  // clock the shift register latch pin high, ready to be set low next time
      counter++;
    }
  }

  timer++;
  us_counter += 10;
  if(timer == 1100){ // all servo timing completed
    update_reg_flag = 1; // allow updates to the timing arrays
  }
  if(timer == 1900){ // getting close to servo start-up again,
    update_reg_flag = 0; // don't allow any new timing array updates
  }
  if(timer == 2000){
    timer=0;
    counter=0;
  }
  sei();
}

void Servotor32::delete_from_sorted_array(byte servo, byte group, signed short pos){
  for(byte i=0; i<servos_active_in_group[group]; i++){ // go through all the servos
    if(servos_sorted[group][i] == servo){ // find its place
       for(signed char j=i; j<servos_active_in_group[group]-1; j++){//move all servos in front of it back by one
         servos_sorted[group][j] = servos_sorted[group][j+1];
       }
       servos_sorted[group][servos_active_in_group[group]-1] = -1; //insert a -1 at the end of the move
       break; //break out of previous for loop, now that the job is done
    }
  }
  active_servos_hex[group] &= ~pin_2_num[servo-group*SERVOS_PER_GROUP];
  servos_active_in_group[group] -= 1;// decrease the number of active servos in the group by 1
}

void Servotor32::add_to_sorted_array(byte servo, byte group, signed short pos){
  for(byte i=0; i<=servos_active_in_group[group]; i++){ // find the servo
     if(servos_sorted[group][i] == -1){ // if no servos yet entered, set as first
       servos_sorted[group][i] = servo; //insert the servo in its sorted place
       break; //stop the for loop, as the job is done
     }
     else{
       if(servo_positions[servos_sorted[group][i]] > pos){ // if this servo should go before this one
         for(signed char j=servos_active_in_group[group]-1; j>=i; j--){// move all others forward one
           servos_sorted[group][j+1] = servos_sorted[group][j];
         }
         servos_sorted[group][i] = servo; //insert the servo in its sorted place
         break;
       }
     }
  }
  active_servos_hex[group] |= pin_2_num[servo-group*SERVOS_PER_GROUP];
  servos_active_in_group[group] += 1;
}

void Servotor32::update_registers_fast(byte servo, signed short pos){
  byte group = servo/8;
  while(update_reg_flag == 0){ // wait for the servos to stop pulsing before updating the timing arrays
    delayMicroseconds(10);
  }
  // ----- put the servo into, or take it out of its sorted array ------
  
  if(pos > 0){ // if the sevo isn't a kill command, then its an add/change
    if(servo_positions[servo] == -1){// if the servo is inactive
      // insert the servo into the array sorted
      add_to_sorted_array(servo,group,pos);
    }
    else{
      // updating the servo. First delete its existing entry, then insert it

      delete_from_sorted_array(servo,group,pos);
      add_to_sorted_array(servo,group,pos);
    }
  }
  else{ // servo is a kill command
    if(servo_positions[servo] != -1){ // make sure its even on first
      delete_from_sorted_array(servo,group,pos);
    }
  }
  
  servo_positions[servo] = pos;
  
  // ----- create timing idicies from servo/group data -------
  
  // clear the timing arrays for fresh start
  for(uint8_t i=0; i<MAX_TIMINGS; i++){
    servo_timings[i] = 0;
    shift_output[i] = 0xFF;
    shift_latch[i] = 0xFF;
  }

  uint8_t counter_index=0;
  uint8_t current_timing=0;
  uint8_t current_shift_output=0; 
  
  for(byte group=0; group<GROUPS; group++){ //go through each group
    if(servos_active_in_group[group] > 0){ // skip it if the group is active, otherwise:
      servo_timings[counter_index] = group_offsets[group];
      shift_output[counter_index] = active_servos_hex[group];
      shift_latch[counter_index] = (1<<group_latches[group]);
      counter_index +=1;
      
      
      //create additional timings
      for(byte i=0; i<servos_active_in_group[group]; i++){ //create the timings for each servo after that, using the previous output
        if(servo_positions[servos_sorted[group][i]] == servo_positions[servos_sorted[group][i-1]]){ // if this servo's time is the same as the last's
          if(i != 0){
            counter_index -= 1; //reverse the index count-up
          }
          else{
            current_shift_output = shift_output[counter_index-1];
            servo_timings[counter_index] = servo_positions[servos_sorted[group][i]]+ group_offsets[group];
            shift_latch[counter_index] = (1<<group_latches[group]);
          }
        }
        else{
          current_shift_output = shift_output[counter_index-1];
          servo_timings[counter_index] = servo_positions[servos_sorted[group][i]]+ group_offsets[group];
          shift_latch[counter_index] = (1<<group_latches[group]);
        }
        
        //subtract the current servo from the shift register output
        current_shift_output &= ~pin_2_num[servos_sorted[group][i]-group*SERVOS_PER_GROUP]; 
        shift_output[counter_index] = current_shift_output;
        counter_index +=1;
      }
    }      
  }
  
}


void Servotor32::printStatus(Stream *serial){
  serial->println("--------------------- Registers ----------------------");
  
  serial->println("Servo Data:");
  serial->println("Servo\tPos\tTimeEnd\t");
  for(byte i=0; i<SERVOS; i++){
    serial->print(i);
    serial->print("\t");
    serial->print(servo_positions[i]);
    serial->println("");
  }
  serial->println("");

  serial->println("Sorted Groups");
  for(byte i=0; i<GROUPS; i++){
    serial->print("Group: ");
    serial->println(i);
    for(byte j=0; j<SERVOS_PER_GROUP; j++){
      serial->print("Servo: ");
      serial->print(servos_sorted[i][j]);
      serial->print("\t");
      serial->println(servo_positions[servos_sorted[i][j]]);
      
    }
  }  

  serial->println("Group Data:");
  serial->println("#\tActive\tHex");
  for(byte i=0; i<GROUPS; i++){
    serial->print(i);
    serial->print("\t");
    serial->print(servos_active_in_group[i]);
    serial->print("\t");
    serial->println(active_servos_hex[i],HEX);
  }
  serial->println("");
  
  serial->println("Timings:");
  serial->println("Pos\tTiming\tOutput\tLatch");
  for(uint8_t i=0; i<MAX_TIMINGS; i++){ // clear existing registers, so they can be cleanly written
    serial->print(i);
    serial->print(":\t");
    serial->print(servo_timings[i]);
    serial->print(",\t");
    serial->print(shift_output[i],HEX);
    serial->print(",\t");
    serial->println(shift_latch[i],HEX);
  }
  serial->println("----------------------------------------------------");
}

// modify the state of a servo
void Servotor32::changeServo(byte servo, short pos){
  if(pos == 0){
    pos = -1;
  }
  if(pos == -1){
    update_registers_fast(servo, pos);
  }
  else{
    update_registers_fast(servo, pos/10);
  }
}

boolean debug = false;
boolean testMode = false;
boolean servoCounting = false;
boolean posCounting = false;
boolean pinCounting = false;

byte numString[6];
int powers[] = {1,10,100,1000};

byte numCount = 0;
unsigned short total = 0;
short inServo = -1;
short inPos = -1;


char pinCommand[3];
uint8_t pinCount=0;

void setPinCommand(){
  if(debug){
    Serial.print("Pin Command - ");
  }

  int pin = -1;
  switch(pinCommand[0]){
    case '0':
      switch(pinCommand[1]){
        case '0':
            pin = 0;
          break;
        case '1':
            pin = 1;
          break;
        case '2':
            pin = 2;
          break;
        case '3':
            pin = 3;
          break;
        case '5':
            pin = 5;
          break;
        case '6':
            pin = 6;
          break;
        case '7':
            pin = 7;
          break;
        case '9':
            pin = 9;
          break;                
      }
      break;
    case '1':
      switch(pinCommand[1]){
        case '0':
            pin = 10;
          break;
        case '1':
            pin = 11;
          break;
        case '2':
            pin = 12;
          break;
        case '3':
            pin = 13;
          break;
          break;
        case '7':
            pin = 17;
          break;
      }
      break;
    case '2':
      switch(pinCommand[1]){
        case '2':
            pin = 22;
          break;
        case '3':
            pin = 23;
          break;
      }
    case 'A':
      switch(pinCommand[1]){
        case '4':
            pin = A4;
          break;
        case '5':
            pin = A5;
          break;          
        case '7':
            pin = A7;
          break;          
        case '8':
            pin = A8;
          break;          
        case '9':
            pin = A9;
          break;
      }
      break;
    case 'R':
      switch(pinCommand[1]){        
        case 'X':
            pin = 0;
          break;    
        case 'L':
            pin = 17;
          break;  
      }
      break;
    case 'T':
      switch(pinCommand[1]){
        case 'X':
          pin = 1;        
          break;           
      }
      break;
  }
  if(debug){
    Serial.print("Set pin ");
    Serial.print(pin);
  }

  if(pin != -1){
    pinMode(pin, OUTPUT);  
    if(pinCommand[2] == 'H'){
      digitalWrite(pin, HIGH);
      if(debug){
        Serial.println(" High");
      }
    }
    if(pinCommand[2] == 'L'){
      digitalWrite(pin, LOW);
      if(debug){
        Serial.println(" Low");
      }
    }
  }
  pinCount = 0;

}


void Servotor32::process(Stream *serial){
  if(serial->available()) { //process input from the serial stream (USB or Serial)
    char inChar = (char)serial->read();
    switch(inChar){
      case '#':
        servoCounting = true;
        numCount = 0;
        inServo = -1;
        inPos = -1;
        break;
      case 'D':
        printStatus(serial);
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
        if(pinCounting){
          // set the pin according to the pin Command
            setPinCommand();
        }
        numCount = 0;
        pinCount = 0;
        break;
      case 'V':
        serial->println("SERVOTOR32_v2.02");
        break;
      case 'C':
        for(int i=0; i<32; i++){
          changeServo(i, 1500);
        }
        serial->println("All Centered");
        break;
      case 'K':
        for(int i=0; i<32; i++){
          changeServo(i,-1);
        }
        serial->println("All Turned Off");
        break;
      case 'L':
        if(servoCounting){
          inServo = tallyCount();
          changeServo(inServo, -1);
          servoCounting = false;
        }
        if(pinCounting){
          if(pinCount < 3){
            pinCommand[pinCount] = inChar;
            pinCount++;
          }          
        }
        break;
      case 'S':
        servoCounting = false;
        posCounting = false;
        pinCounting = true;
        pinCount = 0;
        break;
      default:
        if(pinCounting){
          if(pinCount < 3){
            pinCommand[pinCount] = inChar;
            pinCount++;
          }
        }
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

short Servotor32::tallyCount(){
   total=0;
   for(int i=0; i<numCount; i++){  
     total += powers[i]*numString[(numCount-1)-i];  
   }
   if(numCount == 0){
     total = -1;
   }
   return total;
}

#define MAX_TIME 1000000

float Servotor32::ping(){
  //PB0 for Trigger (17)
  //PB7 for Echo (11)
  
  pinMode(17,OUTPUT);
  pinMode(11,INPUT);

  long duration; 
  float cm;
  digitalWrite(17, LOW); 
  delayMicroseconds(2); 
  digitalWrite(17, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(17, LOW); 
  

  uint8_t bit = digitalPinToBitMask(11);
  uint8_t port = digitalPinToPort(11);
  uint8_t stateMask = (HIGH ? bit : 0);
  
  unsigned long startCount = 0;
  unsigned long endCount = 0;
  unsigned long width = 0; // keep initialization out of time critical area
  
  // convert the timeout from microseconds to a number of times through
  // the initial loop; it takes 16 clock cycles per iteration.
  unsigned long numloops = 0;
  unsigned long maxloops = 500;
	
  // wait for any previous pulse to end
  while ((*portInputRegister(port) & bit) == stateMask)
    if (numloops++ == maxloops)
      return 0;
	
  // wait for the pulse to start
  while ((*portInputRegister(port) & bit) != stateMask)
    if (numloops++ == maxloops)
      return 0;
  
  startCount = micros_new();
  // wait for the pulse to stop
  while ((*portInputRegister(port) & bit) == stateMask) {
    if (numloops++ == maxloops)
      return 0;
    delayMicroseconds(10); //loop 'jams' without this
    if((micros_new() - startCount) > 58000 ){ // 58000 = 1000CM
      return 0;
      break;
    }
  }
  duration = micros_new() - startCount;
  //--------- end pulsein
  cm = (float)duration / 29.0 / 2.0; 
  return cm;
}

float Servotor32::multiPing(unsigned short attempts=5){
  float distances [attempts];
  for(int i=0; i<attempts; i++){
    distances[i] = ping();
  }
  
  // sort them in order
  int i, j;
  float temp;
 
  for (i = (attempts - 1); i > 0; i--)
  {
    for (j = 1; j <= i; j++)
    {
      if (distances[j-1] > distances[j])
      {
        temp = distances[j-1];
        distances[j-1] = distances[j];
        distances[j] = temp;
      }
    }
  }
  
  // return the middle entry
  return distances[(int)ceil((float)attempts/2.0)];
  
}
