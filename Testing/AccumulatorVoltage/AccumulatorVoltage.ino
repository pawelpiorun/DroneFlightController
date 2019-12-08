#include "TM1637Display.h"

#define CLK 2
#define DIO 6

#define IN1S A3
#define IN2S A4
#define IN3S A6
#define IN4S A7

#define displayColon 0b01000000

TM1637Display display(CLK, DIO);

int reading1s = 0;
int reading2s = 0;
int reading3s = 0;
int reading4s = 0;

uint8_t allText[]={
  SEG_A|SEG_B|SEG_C|SEG_E|SEG_F|SEG_G,
  SEG_D|SEG_E|SEG_F,
  SEG_D|SEG_E|SEG_F
};

void setup()
{
  pinMode(IN1S, INPUT);
  pinMode(IN2S, INPUT);
  pinMode(IN3S, INPUT);
  pinMode(IN4S, INPUT);
  
  display.setBrightness(10);
}

void loop()
{
  display.showNumberDecEx(1,0,0);
  reading1s = analogRead(IN1S);
  delay(500);
  int voltage1s = map(reading1s, 0,1024, 0, 500);
  display.showNumberDecEx(voltage1s, displayColon, false);
  delay(1500);

  display.showNumberDecEx(2,0,0);
  reading2s = analogRead(IN2S);
  delay(500);
  int voltage2s = map(reading2s, 0,1024, 0, 500);
  display.showNumberDecEx(voltage2s, displayColon, false);
  delay(1500);

  display.showNumberDecEx(3,0,0);
  reading3s = analogRead(IN3S);
  delay(500);
  int voltage3s = map(reading3s, 0,1024, 0, 500);
  display.showNumberDecEx(voltage3s, displayColon, false);
  delay(1500);

  display.showNumberDecEx(4,0,0);
  reading4s = analogRead(IN4S);
  delay(500);
  int voltage4s = map(reading4s, 0,1024, 0, 500);
  display.showNumberDecEx(voltage4s, displayColon, false);
  delay(1500);

  display.setSegments(allText, 3, 1);
  delay(1000);
  int readingSum = reading1s + reading2s + reading3s + reading4s;
  int mainVoltage = map(readingSum, 0, 4096, 0, 2000);
  display.showNumberDecEx(mainVoltage, displayColon, false);
  delay(2000);
}
