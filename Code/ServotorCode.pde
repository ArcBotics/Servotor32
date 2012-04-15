/***************************************************
~~~~~~~~~~~~~~~~~~~~~ Servitor ~~~~~~~~~~~~~~~~~~~~~
 The Arduino-based, Open-Source 32-Servo Controller
***************************************************/
//import libraries
#include "TimerOne.h"
#include "SPI.h"

//setup pins - http://pjrc.com/teensy/td_digital.html

#define PERIOD 2000 //pulse period in 10's of uS. 20000uS = 20ms = 50hz

//setup timing variables
signed short servoPositions[32];   //array of servo position values [servo]=position in 10's of uSs
unsigned char pulseWidths[201][4]; //array of when to change what pins at what times
                                   //[time in 10's of uS -50uS][servo group]=group bitmask
unsigned short timeIncs = 0;       //number of time increments passed
unsigned char output = 0xFF;
unsigned char bitMaskZeroes[9] = {0x01,0x02,0x04,0x08, 0x10,0x20,0x40,0x80, 0x00};
unsigned char bitMaskOnes[9]   = {0xFE,0xFD,0xFB,0xF7, 0xEF,0xDF,0xBF,0x7F, 0xFF};

unsigned char groupPin[4] = {7,6,5,4};

//used in parsing incoming cmnds
char cmnd[128]; //string containing cmnd charecters read from serial
signed char comPtrW=0; //pointer used in reading incoming command serial data
char current; //current charecter from incoming command serial data
signed char comServoNum=-1; //servo number from cmnd to send to update
signed short comServoPos = -1; //servo position from cmnd to send to update
signed char comPtrR=-1; //pointer used in reading and parsing cmnd

signed char pwmActive = 1;
signed char velocityCalled = 1;

//This line defines a "Uart" object to access the serial port
//for use with communicating with the bluetooth module
HardwareSerial Uart = HardwareSerial();

//keeps track of whether to output serial to bluetooth or usb
//-1=Both, 0=USB, 1=Bluetooth
signed char usingBluetooth = -1;

void setup()
{
  //setup pin modes
  DDRB = 0xFF;  // sets pins B0 to B7 as outputs
  //setup PC serial port
  Serial.begin(115200);
  //setup bluetooth serial port
  Uart.begin(115200);
  
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

void shiftOutSpi(unsigned char outByte, unsigned char chip_latch){
      SPDR = outByte;
      while(!(SPSR & (1<<SPIF)));
      PORTB |= bitMaskZeroes[chip_latch];
      PORTB &= bitMaskOnes[chip_latch];
}

unsigned char group = 0;
unsigned short minPeriod = 50;
unsigned short maxPeriod = 250;
unsigned short switchTime = 260;
#define INCREMENT_TIME 260 //the number of 10s of microseconds between pulse groups
#define LARGEST_GROUP 3 //the largested-numbered group of servos. Largest possible is 

void callback()
{  
  if((pwmActive==0) && (velocityCalled == 0)){
    velocityCallback();
    velocityCalled = 1;
  }
  //update the servos in the appropriate group based on the groups timing array (pulseWidths)
  if((timeIncs >=minPeriod) && (timeIncs <=maxPeriod)){
    if(pulseWidths[timeIncs-minPeriod][group] != 0xFF){ //if there are any changes to Port B
      output &= pulseWidths[timeIncs-minPeriod][group];
      shiftOutSpi(output,groupPin[group]);
      //2-570-770-670-7F
      /*
      if(group==2){
        Serial.print(group,DEC);
        Serial.print("-");
        Serial.print(minPeriod,DEC);
        Serial.print("-");
        Serial.print(maxPeriod,DEC);
        Serial.print("-");
        Serial.print(timeIncs,DEC);
        Serial.print("-");
        Serial.print(output,HEX);
      }
      */
    }
  }
  
  //move to the next servo group, bring the pins high to start that servo period
  if(timeIncs==switchTime){
    if(group==LARGEST_GROUP){ //if its the first group of the cycle
      if(switchTime == 2000){ 
        timeIncs=0;  //reset increment timer
        group=0;
        minPeriod=50;
        maxPeriod=250;
        switchTime=260;
        pwmActive=1; 
        output = 0xFF;
        shiftOutSpi(output,groupPin[group]); //set for the next group high    
      }
      else{
        switchTime=2000;
        pwmActive=0;
        velocityCalled=0;
      }
    }
    else{
      group++; //increment the group
      minPeriod += INCREMENT_TIME;
      maxPeriod += INCREMENT_TIME;
      switchTime += INCREMENT_TIME;
      output = 0xFF;
      shiftOutSpi(output,groupPin[group]); //set for the next group high    
    }
  }
  timeIncs++;
}

void updatePWM(unsigned char servoNum, short servoPos){
  unsigned short tempPosIndex;
  unsigned short servoPosIndex;
  unsigned char group;
  unsigned char groupServoNum;
  
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
  if(servoPos == -1){  //kill the servo
    pulseWidths[tempPosIndex][group] |=  bitMaskZeroes[groupServoNum]; //remove old update position   
    servoPositions[servoNum] = servoPos; //update current servo position
  }
  else{
    pulseWidths[tempPosIndex][group] |=  bitMaskZeroes[groupServoNum]; //remove old update position
    pulseWidths[servoPosIndex][group] &=  bitMaskOnes[groupServoNum]; //add to new update position
    servoPositions[servoNum] = servoPos; //update current servo position
  }
}

void invalid(void){
  Uart.println("invalid");
  Serial.println("invalid");
  comServoNum=-1;
}

unsigned short velocityTimesToInc[32]; //number of times left to increase position
signed short velocityInc[32]; //amount to increase the position each velocity update cycle
signed short velocityOnes[32]; //number of increase-by-ones left used in case of remainders in velocity division

void velocityCallback(void){
  signed short newPos = 0;
  for(unsigned char i=0; i<=31; i++){ //check every servo
    if(velocityTimesToInc[i]!=0){ //if the servo needs updating
      newPos = servoPositions[i] + velocityInc[i]; //increase the position based on the pre-determined velocity
      if(velocityOnes[i] > 0){ //add an extra one position in there if needed (for the remainder)
        newPos += 1;
        velocityOnes[i] = velocityOnes[i]-1;
      }
      if(velocityOnes[i] < 0){
        newPos -= 1;
        velocityOnes[i] = velocityOnes[i]+1;
      }
      velocityTimesToInc[i]--;
      updatePWM(i,newPos); //send update to the servo
    }
  }
}

void moveVelocity(unsigned char servo, unsigned short positionEnd, unsigned short timeToIncrement){
  timeToIncrement /= 20; //carve timeToIncrement into 20ms chunks (minimum change between servo timings)
  velocityTimesToInc[servo] = timeToIncrement;
  if(positionEnd >= servoPositions[servo]){
    velocityInc[servo] = ((positionEnd-servoPositions[servo])/timeToIncrement);
    velocityOnes[servo] = (positionEnd-servoPositions[servo])-velocityInc[servo]*velocityTimesToInc[servo];
  }
  else{
    velocityInc[servo] = -((servoPositions[servo]-positionEnd)/timeToIncrement);
    velocityOnes[servo] = -(servoPositions[servo]-positionEnd+(velocityInc[servo]*velocityTimesToInc[servo]));
  }
}



void loop() //main loop, handles incoming cmnds (and later velocity timing)
{
  if(comPtrW==-1){ //parse cmnd data if its ready
    //barf back cmnd (Debug)
    Serial.print("cmnd:");
    Serial.println(cmnd);
    Uart.print("cmnd:");
    Uart.println(cmnd);
    comPtrR=0;
    comPtrW=0;
    comServoPos=-1;
    comServoNum=-1;
    //check for set servo cmnd
    if(cmnd[comPtrR]=='D'){
      Serial.println("--DEBUG DUMP BEGIN--");
      Serial.println("Non-Empty Timing Registers:");
      Serial.println("Group #, Timing Index, Value (hex)");
      Uart.println("--DEBUG DUMP BEGIN--");
      Uart.println("Non-Empty Timing Registers:");
      Uart.println("Group #, Timing Index, Value (hex)");
      for(unsigned char i=0; i<4; i++){
        for(unsigned char j=0; j<201; j++){
          if(pulseWidths[j][i] != 0xFF){
            Serial.print(int(i));
            Serial.print(" ");
            Serial.print(int(j));
            Serial.print(" ");
            Serial.println(pulseWidths[j][i], HEX);
            Uart.print(int(i));
            Uart.print(" ");
            Uart.print(int(j));
            Uart.print(" ");
            Uart.println(pulseWidths[j][i], HEX);
          }
        }
      }
      Serial.println("Servo Position Registers:");
      Serial.println("Servo, Position (mS)");
      Uart.println("Servo Position Registers:");
      Uart.println("Servo, Position (mS)");
      for(unsigned char i=0; i<32; i++){
        Serial.print(int(i));
        Serial.print(" ");
        Serial.println(servoPositions[i]);
        Uart.print(int(i));
        Uart.print(" ");
        Uart.println(servoPositions[i]);
      }
    
      Serial.println("--DEBUG DUMP END--");  
      Uart.println("--DEBUG DUMP END--");  
    }
    if(cmnd[comPtrR]=='R'){
      for(unsigned char i=0; i<4; i++){
        for(unsigned char j=0; j<201; j++){
          pulseWidths[j][i]=0xFF;
        }
      }  
      for(int i=0; i<32; i++){
        updatePWM(i,servoPositions[i]);
      }
      Serial.println("reset");
      Uart.println("reset");
    }
    if(cmnd[comPtrR]=='#'){ //if you see a #
      if((cmnd[comPtrR+1]>='0')&&(cmnd[comPtrR+1]<='9')){ //look ahead one and check that its a number
        if(cmnd[comPtrR+2] == ' '){ //and if its a number, check there's a space
          comServoNum=cmnd[comPtrR+1]-48; //#+number+space = a single-digit servo number   
          comPtrR+=3;
        }
        else{
          if((cmnd[comPtrR+2]>='0')&&(cmnd[comPtrR+2]<='9')){
            if(cmnd[comPtrR+3] == ' '){  //#+number+number+space = a two-digit servo number
              comServoNum = (cmnd[comPtrR+1]-48)*10+(cmnd[comPtrR+2]-48);
              comPtrR+=4;
            }
            else{  //#+number+number+not space not valid
              invalid(); 
            }
          }
          else{
            //#+not number, #+number+not space, 
            invalid();
          }
        }
      }  
      if(comServoNum != -1){
          //Serial.print("Servo #");
          //Serial.println(comServoNum);    
      }
    }//end check servo command
    //check for position cmnd
    if((cmnd[comPtrR]=='P')||(cmnd[comPtrR]=='p')){
      if((cmnd[comPtrR+1]>='0')&&(cmnd[comPtrR+1]<='9')){
        if((cmnd[comPtrR+2]>='0')&&(cmnd[comPtrR+2]<='9')){
          if((cmnd[comPtrR+3]>='0')&&(cmnd[comPtrR+3]<='9')){
            if((cmnd[comPtrR+4]>='0')&&(cmnd[comPtrR+4]<='9')){
              comServoPos = (cmnd[comPtrR+1]-48)*1000+(cmnd[comPtrR+2]-48)*100+(cmnd[comPtrR+3]-48)*10+(cmnd[comPtrR+4]-48);
              //Serial.print("Position ");
              //Serial.println(comServoPos);
              if((comServoNum >=0)&&(comServoNum <=31)){
                //moveVelocity(comServoNum,comServoPos,2000); 
                updatePWM(comServoNum,comServoPos);
              }
              else{
                invalid();
              }
            }
            else if((cmnd[comPtrR+4]>=' ')||(cmnd[comPtrR+4]<='\0')){
              comServoPos = (cmnd[comPtrR+1]-48)*100+(cmnd[comPtrR+2]-48)*10+(cmnd[comPtrR+3]-48);
              //Serial.print("Position ");
              //Serial.println(comServoPos);
              if((comServoNum >=0)&&(comServoNum <=31)){
                moveVelocity(comServoNum,comServoPos,2000); 
                //updatePWM(comServoNum,comServoPos);
              }
              else{
                invalid();
              }
            }
            else{
              invalid();
            }
          }
        }
      }
    }//end check position command
    if((cmnd[comPtrR]=='Q')||(cmnd[comPtrR]=='q')){
      Serial.print("#");
      Serial.print(comServoNum);
      Serial.print(" P");
      Serial.println(int(servoPositions[comServoNum]));
      Uart.print("#");
      Uart.print(comServoNum);
      Uart.print(" P");
      Uart.println(int(servoPositions[comServoNum]));
    }
    if((cmnd[comPtrR]=='K')||(cmnd[comPtrR]=='k')){
      updatePWM(comServoNum,-1);
      Serial.print("#");
      Serial.print(comServoNum);
      Serial.print(" K");
      Uart.print("#");
      Uart.print(comServoNum);
      Uart.print(" K");
    }
    //command parse ends, 
    comServoNum=-1;
    comServoPos=-1;
    comPtrW=0;
    cmnd[0]='\0';   
  }//end command parse
  else{
    if(Serial.available()){ //otherwise read in new data
      current = Serial.read();
      //Serial.print(current);
      //Uart.print(current);
      if(comPtrW == 128){
        Serial.flush();
        comPtrW = 0;
      }
      if((current == '\n') or (current == '\r')){ //terminate string on newline or carridge return
        cmnd[comPtrW] = '\0';
        comPtrW = -1;
      }
      else{
        cmnd[comPtrW] = current;
        comPtrW++;
      }
    }
    if(Uart.available()){ //otherwise read in new data
      current = Uart.read();
      //Serial.print(current);
      //Uart.print(current);
      if(comPtrW == 128){
        Uart.flush();
        comPtrW = 0;
      }
      if((current == '\n') or (current == '\r')){ //terminate string on newline or carridge return
        cmnd[comPtrW] = '\0';
        comPtrW = -1;
      }
      else{
        cmnd[comPtrW] = current;
        comPtrW++;
      }
    }
  }
}
