//*********************************************************//
//
//  Definitions
//
//*********************************************************//
#define SW_L  9   // digital input from left switch
#define X_L   A3  // analog input pin from left X
#define Y_L   A2  // analog input pin from left Y
#define SW_R  3   // digital pin connected to switch output
#define X_R   A5  // analog input pin from right X output
#define Y_R   A4  // analog input pin from right Y output
#define MAX_JOYSTICK_VALUE 1023

#define refreshPeriod  4000 // us

#define MAX_SPEED 2000
#define MIN_SPEED 1000

//*********************************************************//
//
//    Includes
//
//*********************************************************//
#include <RF24.h>
#include <SPI.h>

//*********************************************************//
//
//  Global variables
//
//*********************************************************//
long nextRefreshTime = refreshPeriod;

int data[4];     // data buffer
int yaw, midYaw, currYaw;
int throttle, midThrottle, currThrottle;
int pitch, midPitch, currPitch;
int roll, midRoll, currRoll;

const int middleValue = MIN_SPEED + (MAX_SPEED - MIN_SPEED) / 2;
const float throttleKI = (float) refreshPeriod / 1000000;
const float signalKI = (float) 5 / 3 * refreshPeriod / 1000000;

RF24 radio(7, 8); // NRF pins - (CE, CS)
const byte address[6] = "00001"; // only one-way, so address of receiver

//*********************************************************//
//
//  Main functions
//
//*********************************************************//
void readData()
{
  currYaw = analogRead(X_L) + MIN_SPEED;
  currThrottle = MAX_JOYSTICK_VALUE - analogRead(Y_L) + MIN_SPEED;
  currPitch = MAX_JOYSTICK_VALUE - analogRead(Y_R) + MIN_SPEED;
  currRoll = analogRead(X_R) + MIN_SPEED;
}

void checkData()
{
  if (currYaw > MAX_SPEED) currYaw = MAX_SPEED;
  if (currPitch > MAX_SPEED) currPitch = MAX_SPEED;
  if (currRoll > MAX_SPEED) currRoll = MAX_SPEED;
  if (currThrottle > MAX_SPEED) currThrottle = MAX_SPEED;

  if (currYaw < MIN_SPEED) currYaw = MIN_SPEED;
  if (currPitch < MIN_SPEED) currPitch = MIN_SPEED;
  if (currRoll < MIN_SPEED) currRoll = MIN_SPEED;
  if (currThrottle < MIN_SPEED) currThrottle = MIN_SPEED;
}

void setAdjustedData()
{
  if (abs(currThrottle - midThrottle) > 30)
    throttle = throttle + (int)((currThrottle - midThrottle) * throttleKI);

  if (throttle > MAX_SPEED) throttle = MAX_SPEED;
  if (throttle < MIN_SPEED) throttle = MIN_SPEED;

  if (abs(currYaw - midYaw) > 30)
  {

    yaw = yaw + (int)((currYaw - midYaw) * signalKI);
    if ((yaw > midYaw && yaw > currYaw)
        || (yaw < midYaw && yaw < currYaw)) yaw = currYaw;
  }
  else
    yaw = middleValue;

  if (abs(currPitch - midPitch) > 30)
  {
    pitch = pitch + (int)((currPitch - midPitch) * signalKI);
    if ((pitch > midPitch && pitch > currPitch)
        || (pitch < midPitch && pitch < currPitch)) pitch = currPitch;
  }
  else
    pitch = middleValue;

  if (abs(currRoll - midRoll) > 30)
  {
    roll = roll + (int)((currRoll - midRoll) * signalKI);
    if ((roll > midRoll && roll > currRoll)
        || (roll < midRoll && roll < currRoll)) roll = currRoll;
  }
  else
    roll = middleValue;
}

void checkStop()
{
  if (digitalRead(SW_R) == LOW && digitalRead(SW_L) == LOW)
    throttle = MIN_SPEED;
}

//*********************************************************//
//
//  Program setup
//
//*********************************************************//
void setup()
{
  //Joystick
  pinMode(SW_L, INPUT_PULLUP);
  pinMode(SW_R, INPUT_PULLUP);

  //nRF
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);  // one of four levels (MIN, LOW, HIGH, MAX)
  radio.setDataRate(RF24_2MBPS);  // or 250KBPS or 1MBPS
  radio.stopListening();          // because only transmitting

  throttle = MIN_SPEED;
  yaw = middleValue;
  pitch = middleValue;
  roll = middleValue;

  readData();
  midThrottle = currThrottle;
  midYaw = currYaw;
  midPitch = currPitch;
  midRoll = currRoll;
}


//*********************************************************//
//
//  Main loop
//
//*********************************************************//
void loop()
{
  // wait for the refresh period to pass
  while (nextRefreshTime - micros() < refreshPeriod) { }
  nextRefreshTime = micros() + refreshPeriod;

  readData();
  checkData();
  setAdjustedData();
  checkStop();

  data[0] = yaw;
  data[1] = throttle;
  data[2] = roll;
  data[3] = pitch;

  radio.write(&data, sizeof(data), false);
}
