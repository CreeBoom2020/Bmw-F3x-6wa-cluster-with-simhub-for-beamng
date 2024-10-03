#ifndef SHCUSTOMPROTOCOL_H
#define SHCUSTOMPROTOCOL_H

#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include "MultiMap.h"
#include "X9C10X.h" 
#include "CRC8.h"
#define lo8(x) ((int)(x)&0xff)
#define hi8(x) ((int)(x)>>8)
#define loo8(x) ((int)(x)&0xff)
#define hii8(x) ((int)(x)>>8)
extern bool GRPMOND;
extern bool EDITEDSELUA;
extern bool checkengwhenoff;
extern int setlanguage;
extern int setcorf;
extern int setmpgl100orkml;
extern int setkmormiles;
extern int backlight;
extern int drivemod;

const int spiCS = 10;  
const int intPin = 2; 

MCP_CAN CAN(spiCS);
CRC8 crc8Calculator;


// create varibles
int Speed = 0;
int RPM = 0;
int Temp = 140;
int count = 0;
int count2 = 0;
String blinkerleft;
String blinkerright;
int cruise;
String parkingbreak;
String oil_warn;
String batWarning = "FALSE";
String lights;
int turnLeft = 0;
int turnRight = 0;
int absWarning = 0;
int dscWarning = 0;
int dscSwitch = 0;
String checkEngine_PCARS = "0";
String Clusterlight = "False";
int pcars_mcarflags = 0;
int acc_lightstage = 0;
int acc_flashlight = 0;
int handbrake = 0;
int turnleft_fs = 0;
int turnright_fs = 0;
String cruise_fs;
String engine_fs;
String lights_fs;
String gear;
int showLights = 0;
int light = 0;
int highbeam = 0;
int fogg = 0;
int brakes = 0;
int EngineIgnitionOn = 0;
int throthel = 0;
int H = 0;
int braketemp = 0;
String Game = "ETS2";
int fuelpercentage = 0; 
int oilpress = 0; 
String checkEngine_ETS;
int rightActive = 0;
int leftActive = 0;
int RPM2 = 0;
int handproc = 0;
int prevhand = 0;
int WTemp = 0;  
int counter4Bit = 0;
int dfl = 0;
int dfr = 0;
int drl = 0;
int drr = 0;
int hood = 0;
int trunk = 0;
int tpms = 0;
int sos = 0;
int dmode = 0;
int manual = 0;
int gas = 0;
int gmax = 0;
int runs = 0;
int runsg = 0;
int runsb = 0;
int tfr = 0;
int tfl = 0;
int trr = 0;
int trl = 0;
int tempomats = 0;
int idleRPM = 0;
int efficient = 0;
int distanceTravelledCounter = 0;
int selectedGear = 0;
int corf;
int mpgl100orkm;
int kmormiles;
int byte2;
int byte3;
int cv;
// CRC-8 Calculation Function only for RPM
uint8_t crc8(uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;  // Initial value
    uint8_t polynomial = 0x1D;  // Polynomial

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];  // XOR the current byte with the CRC

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {  // If the MSB is set
                crc = (crc << 1) ^ polynomial;  // Shift and XOR with the polynomial
            } else {
                crc <<= 1;  // Just shift left
            }
        }
    }

    return crc ^ 0x2C;  // Final XOR with 0x2C
}

class SHCustomProtocol {

public:

void Setup() { 
	    if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) {
        Serial.println("MCP2515 Initialized Successfully!");
        CAN.setMode(MCP_NORMAL);
    } else {
        Serial.println("Error Initializing MCP2515...");
        while (1);  
    }
    crc8Calculator.begin();
    pinMode(intPin, INPUT);
prevhand = 1;
}

void read() {
      Speed = FlowSerialReadStringUntil(';').toInt();                        
      RPM = FlowSerialReadStringUntil(';').toInt();
      Temp = FlowSerialReadStringUntil(';').toInt();
      fuelpercentage = FlowSerialReadStringUntil(';').toInt();
      oilpress = FlowSerialReadStringUntil(';').toInt();
      checkEngine_ETS = FlowSerialReadStringUntil(';');
      blinkerright = FlowSerialReadStringUntil(';');
      blinkerleft = FlowSerialReadStringUntil(';');
      cruise = FlowSerialReadStringUntil(';').toInt();
      parkingbreak = FlowSerialReadStringUntil(';');
      oil_warn = FlowSerialReadStringUntil(';');
      batWarning = FlowSerialReadStringUntil(';');
      lights = FlowSerialReadStringUntil(';');
      turnLeft = FlowSerialReadStringUntil(';').toInt();
      turnRight = FlowSerialReadStringUntil(';').toInt();
      absWarning = FlowSerialReadStringUntil(';').toInt();
      dscWarning = FlowSerialReadStringUntil(';').toInt();
      dscSwitch = FlowSerialReadStringUntil(';').toInt();
      checkEngine_PCARS = FlowSerialReadStringUntil(';');
      pcars_mcarflags = FlowSerialReadStringUntil(';').toInt();
      acc_lightstage = FlowSerialReadStringUntil(';').toInt();
      acc_flashlight =  FlowSerialReadStringUntil(';').toInt();
      handbrake =  FlowSerialReadStringUntil(';').toInt();
      turnleft_fs =  FlowSerialReadStringUntil(';').toInt();
      turnright_fs =  FlowSerialReadStringUntil(';').toInt();
      cruise_fs = FlowSerialReadStringUntil(';');
      engine_fs = FlowSerialReadStringUntil(';');
      lights_fs = FlowSerialReadStringUntil(';');
      gear = FlowSerialReadStringUntil(';');
      light = FlowSerialReadStringUntil(';').toInt();
      fogg = FlowSerialReadStringUntil(';').toInt();
      brakes = FlowSerialReadStringUntil(';').toInt();
      EngineIgnitionOn = FlowSerialReadStringUntil(';').toInt();
      throthel = FlowSerialReadStringUntil(';').toInt();
      H = FlowSerialReadStringUntil(';').toInt();
      braketemp = FlowSerialReadStringUntil(';').toInt();
      highbeam = FlowSerialReadStringUntil(';').toInt();
      WTemp = FlowSerialReadStringUntil(';').toInt();
      gas = FlowSerialReadStringUntil(';').toInt();
      idleRPM = FlowSerialReadStringUntil(';').toInt();
      tfl = FlowSerialReadStringUntil(';').toInt();
      tfr = FlowSerialReadStringUntil(';').toInt();
      trl = FlowSerialReadStringUntil(';').toInt();
      trr = FlowSerialReadStringUntil(';').toInt();
      tempomats = FlowSerialReadStringUntil(';').toInt();
      dfl = FlowSerialReadStringUntil(';').toInt();
      dfr = FlowSerialReadStringUntil(';').toInt();
      drl = FlowSerialReadStringUntil(';').toInt();
      drr = FlowSerialReadStringUntil(';').toInt();
      hood = FlowSerialReadStringUntil(';').toInt();
      trunk = FlowSerialReadStringUntil(';').toInt();
      dmode = FlowSerialReadStringUntil(';').toInt();
      showLights = FlowSerialReadStringUntil(';').toInt();
      Game = FlowSerialReadStringUntil('\n');
}



void Loop() {

// counter4bite ++
counter4Bit++;
if (counter4Bit >= 14) { 
counter4Bit = 0; 
}
 
     
// Temp part
if (Temp > 200) { 
Temp = 200;
}
if(Temp > 0) { // -3 for mor acurate display of oil temp
 Temp -= 3;
}


//ing part and Temp part
if(count == 0x77) {
count = 0;
}
count +=1;
uint8_t ignitionStatus = 0x8A;
        unsigned char ignitionWithoutCRC[] = { 0x80 | counter4Bit, ignitionStatus, 0xDD, 0xF1, 0x01, 0x30, 0x06 };
        uint8_t crc = crc8Calculator.get_crc8(ignitionWithoutCRC, 7, 0x44);
        unsigned char ignitionWithCRC[] = { crc, ignitionWithoutCRC[0], ignitionWithoutCRC[1], ignitionWithoutCRC[2], ignitionWithoutCRC[3], ignitionWithoutCRC[4], ignitionWithoutCRC[5], ignitionWithoutCRC[6] };
    CAN.sendMsgBuf(0x12F, 0, 8, ignitionWithCRC);
unsigned char ingandtemp[8] = {0x0, count, count, 0x00, 0x00, count, count, count}; // ignition for F15 cluster

{
ingandtemp[5] = int((0.983607*Temp) + 51.3169);
CAN.sendMsgBuf(0x3f9, 0, 8, ingandtemp);
}


//if water over 119 then activate engine over heated aleart
if(WTemp >= 119) {

 	      uint8_t engine_overheated[] = { 0x40, 39, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, engine_overheated);
} else {

 	      uint8_t engine_overheated1[] = { 0x40, 39, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, engine_overheated1);
}


// handbrake lamp only sent once because sending multible times cluster not happy
uint8_t handbrake_frame[2] = {0xFE, 0xFF};


  if(handbrake == 1){
    handbrake_frame[0] = 0xFE;
  }else{
    handbrake_frame[0] = 0xFD;
  }
  if (handbrake != prevhand) {
prevhand = handbrake;
handproc = 0;
}
  if(handproc == 0) {
  CAN.sendMsgBuf(0x34F, 0, 2, handbrake_frame);
  handproc = 1;
}


//blinkers
 uint8_t blinkerStatus = (turnLeft == false && turnRight == false) ? 0x80 : (0x81 | turnLeft << 4 | turnRight << 5);
    unsigned char blinkersWithoutCRC[] = { blinkerStatus, 0xF0 };
    CAN.sendMsgBuf(0x1F6, 0, 2, blinkersWithoutCRC);
    
    
//Speed
	       if(Speed > 350) {
	Speed = 350;
}
uint16_t calculatedSpeed = (double)Speed * 64.01;
  unsigned char speedWithoutCRC[] = { 0xC0|counter4Bit, lo8(calculatedSpeed), hi8(calculatedSpeed), (Speed == 0 ? 0x81 : 0x91) };
  unsigned char speedWithCRC[] = { crc8Calculator.get_crc8(speedWithoutCRC, 4, 0xA9), speedWithoutCRC[0], speedWithoutCRC[1], speedWithoutCRC[2], speedWithoutCRC[3] };
  CAN.sendMsgBuf(0x1A1, 0, 5, speedWithCRC);	


//RPM
    if(RPM > 7500) {
	RPM = 7500;
}
if(GRPMOND == true) {
if (RPM < 1000 or RPM == 1000) {
RPM2 = map(RPM, 0, 7500, 0, 6200);
} else if (RPM < 2000 or RPM == 2000) {
RPM2 = map(RPM, 0, 7500, 0, 6000); 
} else if (RPM < 3000 or RPM == 3000) {
RPM2 = map(RPM, 0, 7500, 0, 5900);
} else if (RPM < 4000 or RPM == 4000) {
RPM2 = map(RPM, 0, 7500, 0, 5850); 
} else if (RPM < 5000 or RPM == 5000) {
RPM2 = map(RPM, 0, 7500, 0, 5850);	
} else if (RPM < 6000 or RPM == 6000) {
RPM2 = map(RPM, 0, 7500, 0, 5840); 	
} else if (RPM < 7000 or RPM == 7000) {
RPM2 = map(RPM, 0, 7500, 0, 5840); 
} else if (RPM < 7500 or RPM == 7500) {
RPM2 = map(RPM, 0, 7500, 0, 5840); 
}
}
if(GRPMOND == false) {
RPM2 = RPM;
}

idleRPM += 10;
RPM2 += 4;
if(gas == 0 && Speed > 5 && RPM > idleRPM ) {
efficient = 0xF6;
} else {
efficient = 0xF0;
}

uint8_t rpm_frame[8] = {0xF3, 0x4A, 0x06, 0xC0, efficient, 0xC4, 0xFF, 0xFF   };
    
    rpm_frame[1] = (int(RPM2 * 1.557) & 0xff);
    rpm_frame[2] = (int(RPM2 * 1.557) >> 8);
    uint8_t rpm_crc = crc8(rpm_frame, sizeof(rpm_frame));
    rpm_frame[0] = crc;
  CAN.sendMsgBuf(0xf3, 0, 8, rpm_frame);	


RPM2 -= 4;

    rpm_frame[1] = (int(RPM2 * 1.557) & 0xff);
    rpm_frame[2] = (int(RPM2 * 1.557) >> 8);
    crc = crc8(rpm_frame, sizeof(rpm_frame));
    rpm_frame[0] = crc;
  CAN.sendMsgBuf(0xf3, 0, 8, rpm_frame);	
  

//fuel consumption sometimes moves but not every time
    unsigned char mpgWithoutCRC[] = { throthel | counter4Bit, 0xFF, 0x64, 0x64, 0x64, 0x01, 0xF1 };
    unsigned char mpgWithCRC[] = { crc8Calculator.get_crc8(mpgWithoutCRC, 7, 0xFF), mpgWithoutCRC[0], mpgWithoutCRC[1], mpgWithoutCRC[2], mpgWithoutCRC[3], mpgWithoutCRC[4], mpgWithoutCRC[5], mpgWithoutCRC[6] };
    CAN.sendMsgBuf(0x2C4, 0, 8, mpgWithCRC);
	  

//distance travelld counter
    unsigned char mpg2WithoutCRC[] = { 0xF0|counter4Bit, lo8(distanceTravelledCounter), hi8(distanceTravelledCounter), 0xF2 };
    unsigned char mpg2WithCRC[] = { crc8Calculator.get_crc8(mpg2WithoutCRC, 4, 0xde), mpg2WithoutCRC[0], mpg2WithoutCRC[1], mpg2WithoutCRC[2], mpg2WithoutCRC[3], mpg2WithoutCRC[4] };
    CAN.sendMsgBuf(0x2BB, 0, 5, mpg2WithCRC);
    distanceTravelledCounter += Speed*2.9;
    
//SteeringWheel error clear
    unsigned char steeringColumnWithoutCRC[] = { 0xF0|counter4Bit, 0xFE, 0xFF, 0x14 };
    unsigned char steeringColumnWithCRC[] = { crc8Calculator.get_crc8(steeringColumnWithoutCRC, 4, 0x9E), steeringColumnWithoutCRC[0], steeringColumnWithoutCRC[1], steeringColumnWithoutCRC[2], steeringColumnWithoutCRC[3] };
    CAN.sendMsgBuf(0x2A7, 0, 5, steeringColumnWithCRC);
    
    
//Fuel nedel
  boolean isCarMini = false;
  uint8_t inFuelRange[] = {0, 50, 100};
  uint8_t outFuelRange[] = {22, 7, 3};
  uint8_t fuelQuantityLiters = multiMap<uint8_t>(fuelpercentage, inFuelRange, outFuelRange, 3);
  unsigned char fuelWithoutCRC[] = { (isCarMini ? 0 : hi8(fuelQuantityLiters)), (isCarMini ? 0 : lo8(fuelQuantityLiters)), hi8(fuelQuantityLiters), lo8(fuelQuantityLiters), 0x00 };
  CAN.sendMsgBuf(0x349, 0, 5, fuelWithoutCRC);
  
  
//gear display(A)
  gmax += 1;
if (gmax == 0x09) {
gmax = 0;
}
    
selectedGear = 0x00;
manual = counter4Bit;



if (gear == "1") {
manual = 0x10;
runsg = manual;
manual += gmax;
runs = 0;
} else if(gear == "2") {
manual = 0x20;
runsg = manual;
manual += gmax;
runs = 0;
} else if(gear == "3") {
manual = 0x30;
runsg = manual;
manual += gmax;
runs = 0;
} else if(gear == "4") {
manual = 0x40;
runsg = manual;
manual += gmax;
runs = 0;
} else if(gear == "5") {
manual = 0x50;
runsg = manual;
manual += gmax;	
runs = 0;
} else if(gear == "6") {
manual = 0x60;
runsg = manual;
manual += gmax;	
runs = 0;
} else if(gear == "7") {
manual = 0x70;
runsg = manual;
manual += gmax;	
runs = 0;
} else if(gear == "8") {
manual = 0x80;
runsg = manual;
manual += gmax;
runs = 0;
} else if(gear == "9") {
manual = 0x90;
runsg = manual;
manual += gmax;
runs = 0;	
}

if(gear == "N" or gear == "0" or gear == " " or Game == "ETS2") {
runsb = runsg;
runsb += gmax;
manual = runsb;


runs += 1;

}

if(runs > 20) {
manual = counter4Bit;
}

if ( gear == "R") {
selectedGear = 0x40;
manual = counter4Bit;
}
    unsigned char transmissionWithoutCRC[] = { manual, selectedGear, 0xFC, 0xFF }; //0x20= P, 0x40= R, 0x60= N, 0x80= D, 0x81= DS
    unsigned char transmissionWithCRC[] = { crc8Calculator.get_crc8(transmissionWithoutCRC, 4, 0xD6), transmissionWithoutCRC[0], transmissionWithoutCRC[1], transmissionWithoutCRC[2], transmissionWithoutCRC[3] };
    CAN.sendMsgBuf(0x3FD, 0, 5, transmissionWithCRC);
    
    
//check engine
if(checkengwhenoff == false) {
EngineIgnitionOn = 1;
}
  if(oilpress == 1 or EngineIgnitionOn == 0) {
      uint8_t message2[] = { 0x40, 34, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, message2);
} else {
	      uint8_t message2[] = { 0x40, 34, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, message2);
}


//seat belt indecator
uint8_t seat_belt_indecator[] = { 0x40, 71, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
CAN.sendMsgBuf(0x5c0, 0, 8, seat_belt_indecator);



//dscwarning
if(dscWarning == 1) {
      uint8_t dscWarning[] = { 0x40, 215, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, dscWarning);
} else {
	      uint8_t dscWarning1[] = { 0x40, 215, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, dscWarning1);
}


//send drivingmode
unsigned char modeWithoutCRC[] = { 0xF0|counter4Bit, 0, 0, drivemod, 0x11, 0xC0 };
unsigned char modeWithCRC[] = { crc8Calculator.get_crc8(modeWithoutCRC, 6, 0x4a), modeWithoutCRC[0], modeWithoutCRC[1], modeWithoutCRC[2], modeWithoutCRC[3], modeWithoutCRC[4], modeWithoutCRC[5] };
CAN.sendMsgBuf(0x3A7, 0, 7, modeWithCRC);


//clear ABS error
unsigned char abs1WithoutCRC[] = { 0xF0|counter4Bit, 0xFE, 0xFF, 0x14 };
unsigned char abs1WithCRC[] = { crc8Calculator.get_crc8(abs1WithoutCRC, 4, 0xD8), abs1WithoutCRC[0], abs1WithoutCRC[1], abs1WithoutCRC[2], abs1WithoutCRC[3] };
CAN.sendMsgBuf(0x36E, 0, 5, abs1WithCRC);


//alivecounterSaftey error clear
unsigned char aliveCounterSafetyWithoutCRC[] = { counter4Bit, 0xFF };
CAN.sendMsgBuf(0xD7, 0, 2, aliveCounterSafetyWithoutCRC);


//RestrainAribag error clear
unsigned char restraintWithoutCRC[] = { 0x40 | counter4Bit, 0x40, 0x55, 0xFD, 0xFF, 0xFF, 0xFF };
unsigned char restraintWithCRC[] = { crc8Calculator.get_crc8(restraintWithoutCRC, 7, 0xFF), restraintWithoutCRC[0], restraintWithoutCRC[1], restraintWithoutCRC[2], restraintWithoutCRC[3], restraintWithoutCRC[4], restraintWithoutCRC[5], restraintWithoutCRC[6] };
CAN.sendMsgBuf(0x19B, 0, 8, restraintWithCRC);        


//RestrainSeatbelt error clear and lamp off
unsigned char restraint2WithoutCRC[] = { 0xE0 | counter4Bit, 0xF1, 0xF0, 0xF2, 0xF2, 0xFE };
unsigned char restraint2WithCRC[] = { crc8Calculator.get_crc8(restraint2WithoutCRC, 6, 0x28), restraint2WithoutCRC[0], restraint2WithoutCRC[1], restraint2WithoutCRC[2], restraint2WithoutCRC[3], restraint2WithoutCRC[4], restraint2WithoutCRC[5] };
CAN.sendMsgBuf(0x297, 0, 7, restraint2WithCRC); // clears error
uint8_t Seatbeltlamp[] = {  0x40, 77, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF};
CAN.sendMsgBuf(0x5c0, 0, 8, Seatbeltlamp); //disables lamp


//SOS error clear
sos += 1;
if (sos == 0xff) {
sos = 0;
}
uint8_t SOS[] = {  sos, sos, sos, sos, sos, sos, sos, sos };
CAN.sendMsgBuf(0x2C3, 0, 8, SOS);
    
    
//Tire Presure monitor faliure error clear
tpms += 1;
if(tpms == 0xff) {
tpms = 0;
}
uint8_t TPMS[] = {  tpms, tpms, tpms, tpms, tpms, tpms, tpms, tpms };
CAN.sendMsgBuf(0x368, 0, 8, TPMS);


//enable or disable backlight and fogg highbeam and low beam indecator
if((light == 0) && (highbeam == 0) && (fogg == 1)) {
    uint8_t data1[] = {  0x60, 0x00, 0xf7};
    CAN.sendMsgBuf(0x21A, 0, 3, data1);
       
} else if((light == 0) && (highbeam == 1) && (fogg == 1)) {
    uint8_t data3[] = {  0x67, 0x00, 0xf7};
     CAN.sendMsgBuf(0x21A, 0, 3, data3);
    uint8_t mappedBrightness = map(backlight, 0, 100, 0, 253);
        unsigned char backlightBrightnessWithoutCRC[] = { mappedBrightness, 0xFF };
     CAN.sendMsgBuf(0x202, 0, 2, backlightBrightnessWithoutCRC);
    
} else if((light == 1) && (highbeam == 0) && (fogg == 1)) {
    uint8_t data4[] = {  0x65, 0x00, 0xf7};
     CAN.sendMsgBuf(0x21A, 0, 3, data4);
    uint8_t mappedBrightness = map(backlight, 0, 100, 0, 253);
        unsigned char backlightBrightnessWithoutCRC[] = { mappedBrightness, 0xFF };
     CAN.sendMsgBuf(0x202, 0, 2, backlightBrightnessWithoutCRC);
    
} else if((light == 1) && (highbeam == 0) && (fogg == 0)) {
    uint8_t data5[] = {  0x05, 0x00, 0xf7};
     CAN.sendMsgBuf(0x21A, 0, 3, data5);
    uint8_t mappedBrightness = map(backlight, 0, 100, 0, 253);
        unsigned char backlightBrightnessWithoutCRC[] = { mappedBrightness, 0xFF };
     CAN.sendMsgBuf(0x202, 0, 2, backlightBrightnessWithoutCRC);
    
    
} else if((light == 0) && (highbeam == 1) && (fogg == 0)) {
    uint8_t data7[] = {  0x07, 0x00, 0xf7};
     CAN.sendMsgBuf(0x21A, 0, 3, data7);
    uint8_t mappedBrightness = map(backlight, 0, 100, 0, 253);
        unsigned char backlightBrightnessWithoutCRC[] = { mappedBrightness, 0xFF };
     CAN.sendMsgBuf(0x202, 0, 2, backlightBrightnessWithoutCRC);
   } else {
	    uint8_t data8[] = {  0x00, 0x00, 0xf7};
     CAN.sendMsgBuf(0x21A, 0, 3, data8); 
}


//set units
//76 km/l km
//66 mpg km
//89 l/100 km
//206 km/l mi
//147 mpg mi
// l/100  mi
// 18 c°
// 34 f°
if(setcorf == 1) {
byte2 = 18;
} else {
byte2 = 34;
}
if(setmpgl100orkml == 1 && setkmormiles == 1) {
byte3 = 89;
} else if(setmpgl100orkml == 2 && setkmormiles == 1) {
byte3 =	66;
} else if(setmpgl100orkml == 3 && setkmormiles == 1) {
byte3 = 76;
} else if(setmpgl100orkml == 1 && setkmormiles != 1) {
byte3 = 145;
} else if(setmpgl100orkml == 2 && setkmormiles != 1) {
byte3 = 147;
} else if(setmpgl100orkml == 3 && setkmormiles != 1) {
byte3 = 206;
}
  uint8_t cel[] = {  setlanguage, byte2, byte3, 0x00, 0x00, 0x00, 0x00, 0x00};
    CAN.sendMsgBuf(0x291, 0, 8, cel);

//code part where u need the partch

if(EDITEDSELUA == true) {
// cruisecontrol
  if(cruise == 1) {
cv = 0x96;
} else {
cv = 0x00;
}
  unsigned char cruiseWithoutCRC[] = { 0xF0|counter4Bit, 0x00, 0xE0, 0xE1, cv, 0x14, 0x00  };
  unsigned char cruiseWithCRC[] = { crc8Calculator.get_crc8(cruiseWithoutCRC, 7, 0x82), cruiseWithoutCRC[0], cruiseWithoutCRC[1], cruiseWithoutCRC[2], cruiseWithoutCRC[3], cruiseWithoutCRC[4], cruiseWithoutCRC[5], cruiseWithoutCRC[6] };
 CAN.sendMsgBuf(0x289, 0, 8, cruiseWithCRC);
 
 
//tire flat
if (tfl == 1) {
  uint8_t tfld[] = { 0x40, 0x8B, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, tfld);
} else {
  uint8_t tfld[] = { 0x40, 0x8B, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, tfld);
 }

if (tfr == 1) {
  uint8_t tfrd[] = { 0x40, 0x8F, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, tfrd);
} else {
  uint8_t tfrd[] = { 0x40, 0x8F, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, tfrd);
 }
 
 if (trl == 1) {
  uint8_t trld[] = { 0x40, 0x8D, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, trld);
} else {
  uint8_t trld[] = { 0x40, 0x8D, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, trld);
 }
 
 if (trr == 1) {
  uint8_t trrd[] = { 0x40, 0x8C, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, trrd);
} else {
  uint8_t trrd[] = { 0x40, 0x8C, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, trrd);
 }


//doors, trunk, frunk open or closed
if (dfl == 1) {
  uint8_t dfld[] = { 0x40, 0xF, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, dfld);
} else {
  uint8_t dfld[] = { 0x40, 0xF, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, dfld);
 }

if (dfr == 1) {
  uint8_t dfrd[] = { 0x40, 0xE, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, dfrd);
} else {
  uint8_t dfrd[] = { 0x40, 0xE, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, dfrd);
 }
 
 if (drl == 1) {
  uint8_t drld[] = { 0x40, 0x10, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, drld);
} else {
  uint8_t drld[] = { 0x40, 0x10, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, drld);
 }
 
 if (drr == 1) {
  uint8_t drrd[] = { 0x40, 0x11, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, drrd);
} else {
  uint8_t drrd[] = { 0x40, 0x11, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, drrd);
 }

 if (hood == 1) {
  uint8_t hoodd[] = { 0x40, 0x12, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, hoodd);
} else {
  uint8_t hoodd[] = { 0x40, 0x12, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, hoodd);
 }
 
 if (trunk == 1) {
  uint8_t trunkd[] = { 0x40, 0x13, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, trunkd);
} else {
  uint8_t trunkd[] = { 0x40, 0x13, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, trunkd);
 }
 // other found ids but dont used jet
 /*
    uint8_t dataa[] = {  A, A, A, A, A, A, A, A}; 2 = on 1 = off
    sendCANMessage(0x36A, dataa, 8); // Automatic high beam light


  uint8_t dataa[] = {  1AA, 1AA, 1AA, 1AA, 1AA, 1AA, 1AA, 1AA}; 1AA = on E6= off
    sendCANMessage(0x30B, dataa, 8); // Auto start stop /
    */
}
}
};
#endif // __SHCUSTOMPROTOCOL_H__
