#include "Arduino.h"
#include <SPI.h>

/*Connections:
MCP4922   ESP8266
V_DD      3v3
V_REF     1.6V from voltage divider
AVSS      GND
SCK       GPIO14 (D5)
SDI       GPIO13 (D7)
CS        GPIO15 (D8)
LDAC      GPIO5
 */

 
//PIN definitions
#define CS_PIN 15 
#define LDAC_PIN 5

#define PIN_LASER 16

//Definition of a Laser frame
const int MaxFrameSize = 500;

struct Frame {
  int size;
  int16_t x[MaxFrameSize];
  int16_t y[MaxFrameSize];
  //byte laseronoff[MaxFrameSize];
};

Frame g_origFrame;               //Original frame of a horizontal arrow
Frame g_arrFrames[2];            //ISR shows one frame, the main loop updates the other
volatile Frame* g_ptrCurFrame = &g_origFrame;  //frame that should be drawn by the ISR 
int g_isr_cur_frame_idx = 0; //idx of a point as used in the isr routine
////////////////////////////

//Look-up table of Cos and Sin for speed
int tblCos[360];
int tblSin[360];

void setup() {
  // put your setup code here, to run once:

  //set up pins and SPI
  pinMode(PIN_LASER, OUTPUT);
  digitalWrite(PIN_LASER, 0);
  
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setFrequency(20000000); // 20 MHz 
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  pinMode(LDAC_PIN, OUTPUT);
  digitalWrite(LDAC_PIN, HIGH);

  SPIClass__setDataBits(8);  //optimization for speed: preset the data bits
  
  pinMode(0, OUTPUT);
  digitalWrite(0, 0);

  Serial.begin(9600);

  //set up look-up tables
  setupTrigTable();

  //create a drawing
  setupOrigFrame(g_origFrame);
   
  timer1_attachInterrupt(my_timer_isr);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);  // 80/16 = 5MHz
  timer1_write(200); //start soon, call in a loop at 25KHz
    
  digitalWrite(PIN_LASER, 1); //for now, the laser is always on
}


void my_timer_isr() {

 if(++g_isr_cur_frame_idx >= g_ptrCurFrame->size) g_isr_cur_frame_idx = 0;
    
 const int16_t x_pos = g_ptrCurFrame->x[g_isr_cur_frame_idx];
 const int16_t y_pos = g_ptrCurFrame->y[g_isr_cur_frame_idx];
 setDAC(16*x_pos, 16*y_pos);
  
}


int g_angle = 0;

int g_curFrameIdx = 0;

void loop() {

  //Use this for testing the corners
  //setDAC(0,0); delay(5000); setDAC(4095, 0); delay(5000); setDAC(0, 4095); delay(5000); setDAC(4095, 4095); delay(5000); return;

  //draw the new frame
  int newFrameIdx = 1 - g_curFrameIdx;
  Frame& newFrame = g_arrFrames[newFrameIdx];
  newFrame.size = g_origFrame.size;
  for(int i=0; i<g_origFrame.size; ++i) {
    rotatePoint(g_origFrame.x[i], g_origFrame.y[i], 
                newFrame.x[i], newFrame.y[i], 
                128, 128, g_angle); yield();
  }

  //switch the frames
  g_curFrameIdx = newFrameIdx;
  g_ptrCurFrame = g_arrFrames + g_curFrameIdx;

  //update the angle
  g_angle += 5;
  if(g_angle >= 360) g_angle -= 360;
  delay(27);
  

}

void setupTrigTable() {
  for(int16_t angle=0; angle<360; ++angle) {
    double dAngle = angle * 3.14159 / 180;
    tblCos[angle] = 100*cos(dAngle);
    tblSin[angle] = 100*sin(dAngle);
  }
}

inline 
void rotatePoint(const int16_t x, const int16_t y, int16_t& new_x, int16_t& new_y, const int16_t x0, const int16_t y0, int16_t angle) {
  new_x = x0 + (x-x0)*tblCos[angle]/100 + (y-y0)*tblSin[angle]/100;
  new_y = y0 - (x-x0)*tblSin[angle]/100 + (y-y0)*tblCos[angle]/100;
}


void setupOrigFrame(Frame& curFrame) {

  int counter = 0;
  
  for(int i=0; i<10; ++i) { curFrame.x[counter] = 64; curFrame.y[counter] = 128 + i; ++counter; }
  
  for(int i=0; i<100; ++i) { curFrame.x[counter] = 64+i; curFrame.y[counter] = 128+10; ++counter; }
  
  for(int i=0; i<15; ++i) { curFrame.x[counter] = 64+99-i; curFrame.y[counter] = 128+10+i; ++counter; }
  
  for(int i=0; i<44; ++i) { curFrame.x[counter] = 64+99-14+i; curFrame.y[counter] = 128+10+14-24*i/43; ++counter; }
  
  for(int i=0; i<44; ++i) { curFrame.x[counter] = 64+99+29-i; curFrame.y[counter] = 128-24*i/43; ++counter; }
  
  for(int i=0; i<15; ++i) { curFrame.x[counter] = 64+99-14+i; curFrame.y[counter] = 128-24+i; ++counter; }
  
  for(int i=0; i<100; ++i) { curFrame.x[counter] = 64+99-i; curFrame.y[counter] = 128-10; ++counter; }
  
  for(int i=0; i<10; ++i) { curFrame.x[counter] = 64; curFrame.y[counter] = 128-10 + i; ++counter; }

  curFrame.size = counter;

  Serial.print("curFrame.size ="); Serial.println(curFrame.size);
  if(curFrame.size > MaxFrameSize) {
    Serial.println("curFrame.size > MaxFrameSize"); delay(10000);
  }
  
}


///////////////////////////////
//MCP4922 DAC support functions

//Optimization for speed. 
//Source .arduino15/packages/esp8266/hardware/esp8266/2.3.0/libraries/SPI/SPI.cpp, line 222
inline void SPIClass__setDataBits(uint16_t bits) {
    const uint32_t mask = ~((SPIMMOSI << SPILMOSI) | (SPIMMISO << SPILMISO));
    bits--;
    SPI1U1 = ((SPI1U1 & mask) | ((bits << SPILMOSI) | (bits << SPILMISO)));
}

//SPIClass__setDataBits(8); should be already called
inline void SPI_transfer8_fast(uint8_t data) {
  
  while(SPI1CMD & SPIBUSY) {}

  // reset to 8Bit mode
  //setDataBits(8);
  SPI1W0 = data;
  SPI1CMD |= SPIBUSY;
  while(SPI1CMD & SPIBUSY) {}

}

//Based on info in
//https://github.com/helgenodland/MCP4922-Arduino-SPI-Library/blob/master/MCP4922.cpp
inline void setDAC(const uint16_t A, const uint16_t B) {
  //Bit 1 from the left: channel 0 or 1
  //Bit 2 from the left: 1 means VREF is buffered (can use simple voltage divider)
  //Bit 3 from the left: 0 means G = 2, so that VOUT = 0 .. 2*VREF
  //Bit 4 from the left: 1 means the shutdown is controlled by !SHDN pin
  const uint16_t channelA = A | 0b0101000000000000; //const uint16_t channelA = A | 0b0111000000000000;
  const uint16_t channelB = B | 0b1101000000000000; //const uint16_t channelB = B | 0b1111000000000000;
         
  GPOC = 1<<CS_PIN; //digitalWrite(CS_PIN, LOW);
  SPI_transfer8_fast(highByte(channelA)); //SPI.transfer(highByte(channelA));
  SPI_transfer8_fast(lowByte(channelA));  //SPI.transfer(lowByte(channelA));
  GPOS = 1<<CS_PIN; //digitalWrite(CS_PIN, HIGH);
  
  __asm__("nop\n\t");
  
  GPOC = 1<<CS_PIN; //digitalWrite(CS_PIN, LOW);
  SPI_transfer8_fast(highByte(channelB)); //SPI.transfer(highByte(channelB));
  SPI_transfer8_fast(lowByte(channelB));  //SPI.transfer(lowByte(channelB));
  GPOS = 1<<CS_PIN; //digitalWrite(CS_PIN, HIGH); 
  
  GPOC = 1<<LDAC_PIN; //digitalWrite(LDAC_PIN,LOW); 

  __asm__("nop\n\t");

  GPOS = 1<<LDAC_PIN; //digitalWrite(LDAC_PIN,HIGH);
}





