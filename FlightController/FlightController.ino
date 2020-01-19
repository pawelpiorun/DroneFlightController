//=======================================//
//              DEFINITIONS              //
//=======================================//

#define   AUTO_LEVEL      false

#define   MAX_THROTTLE    1500
#define   MIN_SPEED       1000
#define   MAX_SPEED       2000
#define   IDLE_SPEED      1100

#define   escPinRR        4
#define   escPinRL        3
#define   escPinFL        10
#define   escPinFR        9

#define   redPin          6
#define   greenPin        A1
#define   bluePin         1

#define   batteryPin      A6
#define   MIN_BAT_ON_VOLT 80000
#define   MAX_MEAS_VOLT   12650
#define   MIN_BAT_VOLT    10000
#define   MAX_BAT_VOLT    12600
#define   DIODE_MIN_VOLT  800

#define   radioPinCE      7
#define   radioPinCS      8

#define   refreshPeriod   4000 // us

#define   rollKP          1.3f
#define   rollKI          0.04f
#define   rollKD          18.0f
#define   pitchKP         rollKP
#define   pitchKI         rollKI
#define   pitchKD         rollKD
#define   yawKP           4.0f
#define   yawKI           0.02f
#define   yawKD           0.0f
#define   DEAD_BAND       8

//=======================================//
//              INCLUDES                 //
//=======================================//
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>

//=======================================//
//              GLOBALS                  //
//=======================================//

// Timing stuff
unsigned long timer = 0;
unsigned long nextRefreshTime = refreshPeriod;

// Receiver
RF24 radio(radioPinCE, radioPinCS);
const byte txAdress[6] = "00001";      // only one-way communication
int receiverYaw, receiverThrottle, receiverPitch, receiverRoll;
int receiverData[4];

// Gyro
byte gyroAddress = 0x68;
int gyroRoll, gyroPitch, gyroYaw;
double gyroRollOffset, gyroPitchOffset, gyroYawOffset;
bool gyroCalibrationDone = false;

// Accelerometer
int accX, accY, accZ;
double accVector;
float anglePitchAcc, angleRollAcc;

// Angles
boolean gyroAnglesSet = false;
float angleRoll, anglePitch, angleYaw;
float rollLevelAdjust, pitchLevelAdjust;

// ESCs
int escPulseRR, escPulseFR, escPulseRL, escPulseFL;

// PID
float maxPidSignal = 400;
float pitchSetpoint, rollSetpoint, yawSetpoint;
float rollTarget, pitchTarget, yawTarget;
float rollError, pitchError, yawError;
float rollIntegral, pitchIntegral, yawIntegral;
float rollDerivative, pitchDerivative, yawDerivative;
float rollPID, pitchPID, yawPID;

// Battery
int batteryVoltage; // mV
float voltageFactor = MAX_MEAS_VOLT / 1023;
int diodeCompensation = DIODE_MIN_VOLT / voltageFactor;
float filterF1 = 0.92; float filterF2 = 0.08;

// State
bool start = false;
bool isReady = false;

void setup()
{
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  
  pinMode(escPinFL, OUTPUT);
  pinMode(escPinFR, OUTPUT);
  pinMode(escPinRR, OUTPUT);
  pinMode(escPinRL, OUTPUT);
  
  pinMode(batteryPin, INPUT);

  // indicate startup
  digitalWrite(bluePin, LOW);
  digitalWrite(greenPin, HIGH);
  digitalWrite(redPin, HIGH);

  radio.begin();
  radio.openReadingPipe(0, txAdress);
  radio.setPALevel(RF24_PA_MIN);      //one of four levels (MIN, LOW, HIGH, MAX)
  radio.setDataRate(RF24_2MBPS);      // or 250KBPS or 2Mbps
  radio.startListening();

  Wire.begin();
  TWBR = 12;
  startGyro();
  calibrateESC();
  calibrateGyro();
  gyroCalibrationDone = true;
  batteryVoltage = (analogRead(batteryPin) + diodeCompensation) * voltageFactor;
  waitForReceiver();

  start = false;
  isReady = false;

  timer = micros();
  setupFinished();
}

void setupFinished()
{
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(bluePin, HIGH);
    digitalWrite(greenPin, HIGH);
    digitalWrite(redPin, HIGH);
    delay(250);
    digitalWrite(bluePin, LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(redPin, LOW);
    delay(250);
  }
  digitalWrite(greenPin, HIGH);
  delay(250);
}

void loop()
{
  calculateAngles();
  checkStartStopReady();

  calculatePIDsetpoints();
  calculatePIDsignals();
  checkBattery();
  calculatePulses();

  if (nextRefreshTime - micros() > refreshPeriod + 50) digitalWrite(redPin, HIGH);

  // wait for the refresh period to pass
  while (nextRefreshTime - micros() < refreshPeriod) { }
  nextRefreshTime = micros() + refreshPeriod;

  escPulseOutput();
}

void escPulseOutput(int pulse)
{
  escPulseFL = pulse;
  escPulseFR = pulse;
  escPulseRR = pulse;
  escPulseRL = pulse;
  escPulseOutput();
}

void escPulseOutput()
{
  unsigned long timeStamp = micros();
  bool FLdone = false, FRdone = false, RRdone = false, RLdone = false;
  PORTD |= B00011000;   // set D3 (PD3) (RL) and D4 (PD4) (RR) to high
  PORTB |= B00000110;   // set D9 (PB1) (FR) and D10 (PB2) (FL) to high
  unsigned long pulseOverRR = escPulseRR + timeStamp;
  unsigned long pulseOverRL = escPulseRL + timeStamp;
  unsigned long pulseOverFR = escPulseFR + timeStamp;
  unsigned long pulseOverFL = escPulseFL + timeStamp;

  // meanwhile read signals, because we have 1000us spare microseconds
  readSignals();

  while (!(FLdone && FRdone && RRdone && RLdone))
  {
    timeStamp = micros();
    if (timeStamp >= pulseOverRR)
    {
      PORTD &= B11101111;
      RRdone = true;
    }
    if (timeStamp >= pulseOverRL)
    {
      PORTD &= B11110111;
      RLdone = true;
    }
    if (timeStamp >= pulseOverFR)
    {
      PORTB &= B11111101;
      FRdone = true;
    }
    if (timeStamp >= pulseOverFL)
    {
      PORTB &= B11111011;
      FLdone = true;
    }
  }
}

void startGyro()
{
  Wire.beginTransmission(gyroAddress);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(gyroAddress);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  Wire.beginTransmission(gyroAddress);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(gyroAddress);
  Wire.write(0x1B);
  Wire.endTransmission();
  Wire.requestFrom(gyroAddress, 1);
  while (Wire.available() < 1);
  if (Wire.read() != 0x08)
  {
    digitalWrite(redPin, HIGH);
    while (1) delay(10);
  }

  Wire.beginTransmission(gyroAddress);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
}

void calibrateGyro()
{
  for (int i = 0; i < 2000; i++)
  {
    if (i % 25 == 0)
      digitalWrite(bluePin, !digitalRead(bluePin));
    readSignals();
    gyroRollOffset += gyroRoll;
    gyroPitchOffset += gyroPitch;
    gyroYawOffset += gyroYaw;

    escPulseOutput(1000);
    delay(3);
  }

  gyroRollOffset /= 2000;
  gyroPitchOffset /= 2000;
  gyroYawOffset /= 2000;
}

void calibrateESC()
{
  for (int i = 0; i < 1250; i++)
  {
    escPulseOutput(1000);
    delay(3);
  }
}

void readSignals()
{
  Wire.beginTransmission(gyroAddress);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(gyroAddress, 14);
  while (Wire.available() < 14);

  readReceiver();

  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();
  byte temperature = Wire.read() << 8 | Wire.read();
  gyroPitch = Wire.read() << 8 | Wire.read();
  gyroRoll = Wire.read() << 8 | Wire.read();
  gyroYaw = Wire.read() << 8 | Wire.read();

  if (gyroCalibrationDone)
  {
    gyroRoll -= gyroRollOffset;
    gyroPitch -= gyroPitchOffset;
    gyroYaw -= gyroYawOffset;
  }
}

void readReceiver()
{
  if (radio.available())
  {
    radio.read(&receiverData, sizeof(receiverData));
    receiverYaw = receiverData[0];
    receiverThrottle = receiverData[1];
    receiverRoll = receiverData[2];
    receiverPitch = receiverData[3];
    checkReceiverData();
  }
}

void checkReceiverData()
{
  if (receiverYaw < 1000) receiverYaw = 1000;
  if (receiverPitch < 1000) receiverPitch = 1000;
  if (receiverRoll < 1000) receiverRoll = 1000;
  if (receiverThrottle < MIN_SPEED) receiverThrottle = MIN_SPEED;

  if (receiverYaw > 2000) receiverYaw = 2000;
  if (receiverPitch > 2000) receiverPitch = 2000;
  if (receiverRoll > 2000) receiverRoll = 2000;
  if (receiverThrottle > MAX_SPEED) receiverThrottle = MAX_SPEED;
}

void checkBattery()
{
  int currentFactor = (analogRead(batteryPin) + diodeCompensation) * filterF2 * voltageFactor;
  int previousFactor = batteryVoltage * filterF1;
  batteryVoltage = previousFactor + currentFactor;

  if (batteryVoltage < MIN_BAT_VOLT && batteryVoltage > MIN_BAT_ON_VOLT)
    digitalWrite(redPin, HIGH);
}

void waitForReceiver()
{
  int k = 0;
  while (receiverThrottle > 1020 && receiverYaw < 1400)
  {
    readReceiver();
    k++;
    escPulseOutput(1000);
    delay(3);
    if (k % 125 == 0) digitalWrite(greenPin, !digitalRead(greenPin));
  }
}

void checkStartStopReady()
{
  if (receiverThrottle < 1050 && receiverYaw < 1050) isReady = true;
  //When yaw stick is back in the center position start the motors (step 2).
  if (isReady && receiverThrottle < 1050 && receiverYaw > 1450)
  {
    start = true;

    anglePitch = anglePitchAcc;
    angleRoll = angleRollAcc;
    gyroAnglesSet = true;

    //Reset the PID controllers for a bumpless start.
    rollIntegral = 0;
    rollDerivative = 0;
    pitchIntegral = 0;
    pitchDerivative = 0;
    yawIntegral = 0;
    yawDerivative = 0;
    rollError = 0;
    pitchError = 0;
    yawError = 0;
  }

  if (start && receiverThrottle < 1050 && receiverYaw > 1950)
  {
    start = false;
    isReady = false;
  }
}

void calculatePulses()
{
  if (start)
  {
    if (receiverThrottle > MAX_THROTTLE) receiverThrottle = MAX_THROTTLE;
    escPulseFL = receiverThrottle - pitchPID - rollPID + yawPID;
    escPulseFR = receiverThrottle - pitchPID + rollPID - yawPID;
    escPulseRR = receiverThrottle + pitchPID + rollPID + yawPID;
    escPulseRL = receiverThrottle + pitchPID - rollPID - yawPID;

    // If battery connected, compensate for voltage drop
    if (batteryVoltage < MAX_BAT_VOLT && batteryVoltage > MIN_BAT_ON_VOLT)
    {
      escPulseFL += escPulseFL * ((MAX_BAT_VOLT - batteryVoltage) / (float)3500);
      escPulseFR += escPulseFR * ((MAX_BAT_VOLT - batteryVoltage) / (float)3500);
      escPulseRR += escPulseRR * ((MAX_BAT_VOLT - batteryVoltage) / (float)3500);
      escPulseRL += escPulseRL * ((MAX_BAT_VOLT - batteryVoltage) / (float)3500);
    }

    if (escPulseFL < IDLE_SPEED) escPulseFL = IDLE_SPEED;
    if (escPulseFR < IDLE_SPEED) escPulseFR = IDLE_SPEED;
    if (escPulseRR < IDLE_SPEED) escPulseRR = IDLE_SPEED;
    if (escPulseRL < IDLE_SPEED) escPulseRL = IDLE_SPEED;

    if (escPulseFL > MAX_SPEED) escPulseFL = MAX_SPEED;
    if (escPulseFR > MAX_SPEED) escPulseFR = MAX_SPEED;
    if (escPulseRR > MAX_SPEED) escPulseRR = MAX_SPEED;
    if (escPulseRL > MAX_SPEED) escPulseRL = MAX_SPEED;
  }
  else
  {
    escPulseFL = MIN_SPEED;
    escPulseFR = MIN_SPEED;
    escPulseRR = MIN_SPEED;
    escPulseRL = MIN_SPEED;
  }
}

// TODO - understand
void calculateAngles()
{
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  anglePitch += gyroPitch * 0.0000611;
  angleRoll += gyroRoll * 0.0000611;

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  anglePitch -= anglePitch * sin(gyroYaw * 0.000001066);
  angleRoll += angleRoll * sin(gyroYaw * 0.000001066);

  //Accelerometer angle calculations
  accVector = sqrt((accX * accX) + (accY * accY) + (accZ * accZ));

  if (abs(accY) < accVector)
    anglePitchAcc = asin((float)accY / accVector) * 57.296;

  if (abs(accX) < accVector)
    angleRollAcc = asin((float)accX / accVector) * -57.296;

  // TODO?
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  anglePitchAcc -= 0.0;        //Accelerometer calibration value for pitch.
  angleRollAcc -= 0.0;         //Accelerometer calibration value for roll.

  anglePitch = anglePitch * 0.9996 + anglePitchAcc * 0.0004;
  angleRoll = angleRoll * 0.9996 + angleRollAcc * 0.0004;

  pitchLevelAdjust = anglePitch * 15;                 //Calculate the pitch angle correction
  rollLevelAdjust = angleRoll * 15;                   //Calculate the roll angle correction

  if (!AUTO_LEVEL)
  {
    pitchLevelAdjust = 0;
    rollLevelAdjust = 0;
  }
}

// TODO - understand
void calculatePIDsetpoints()
{
  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  rollSetpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiverRoll > 1500 + DEAD_BAND)
    rollSetpoint = receiverRoll - 1500 - DEAD_BAND;
  else if (receiverRoll < 1500 - DEAD_BAND)
    rollSetpoint = receiverRoll - 1500 + DEAD_BAND;

  rollSetpoint -= rollLevelAdjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
  rollSetpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.

  pitchSetpoint = 0;
  if (receiverPitch > 1500 + DEAD_BAND)
    pitchSetpoint = receiverPitch - 1500 - DEAD_BAND;
  else if (receiverPitch < 1500 - DEAD_BAND)
    pitchSetpoint = receiverPitch - 1500 + DEAD_BAND;

  pitchSetpoint -= pitchLevelAdjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
  pitchSetpoint /= 3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  yawSetpoint = 0;
  if (receiverThrottle > 1050)
  {
    if (receiverYaw > 1500 + DEAD_BAND)
      yawSetpoint = (receiverYaw - 1500 - DEAD_BAND) / 3.0;
    else if (receiverYaw < 1500 - DEAD_BAND)
      yawSetpoint = (receiverYaw - 1500 + DEAD_BAND) / 3.0;
  }
}

// TODO - understand
void calculatePIDsignals()
{
  // Gyro input
  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  rollTarget = (rollTarget * 0.7) + ((gyroRoll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  pitchTarget = (pitchTarget * 0.7) + ((gyroPitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  yawTarget = (yawTarget * 0.7) + ((gyroYaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

  //Roll calculations
  float lastRollError = rollError;
  rollError = rollTarget - rollSetpoint;
  rollIntegral += rollKI * rollError;
  rollDerivative = rollKD * (rollError - lastRollError);

  //Pitch calculations
  float lastPitchError = pitchError;
  pitchError = pitchTarget - pitchSetpoint;
  pitchIntegral += pitchKI * pitchError;
  pitchDerivative = pitchKD * (pitchError - lastPitchError);

  //Yaw calculations
  float lastYawError = yawError;
  yawError = yawTarget - yawSetpoint;
  yawIntegral += yawKI * yawError;
  yawDerivative = yawKD * (yawError - lastYawError);

  validateIsignals();

  rollPID = rollKP * rollError + rollIntegral + pitchDerivative;
  pitchPID = pitchKP * pitchError + pitchIntegral + pitchDerivative;
  yawPID = yawKP * yawError + yawIntegral + yawDerivative;

  validatePIDsignals();
}

void validateIsignals()
{
  if (rollIntegral > maxPidSignal)
    rollIntegral = maxPidSignal;
  else if (rollIntegral < maxPidSignal * -1)
    rollIntegral = maxPidSignal * -1;


  if (pitchIntegral > maxPidSignal)
    pitchIntegral = maxPidSignal;
  else if (pitchIntegral < maxPidSignal * -1)
    pitchIntegral = maxPidSignal * -1;

  if (yawIntegral > maxPidSignal)
    yawIntegral = maxPidSignal;
  else if (yawIntegral < maxPidSignal * -1)
    yawIntegral = maxPidSignal * -1;
}

void validatePIDsignals()
{
  if (rollPID > maxPidSignal)
    rollPID = maxPidSignal;
  else if (rollPID < maxPidSignal * -1)
    rollPID = maxPidSignal * -1;

  if (pitchPID > maxPidSignal)
    pitchPID = maxPidSignal;
  else if (pitchPID < maxPidSignal * -1)
    pitchPID = maxPidSignal * -1;

  if (yawPID > maxPidSignal)
    yawPID = maxPidSignal;
  else if (yawPID < maxPidSignal * -1)
    yawPID = maxPidSignal * -1;

}
