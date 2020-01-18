#define refreshPeriod  4000 // us

#define MAX_THROTTLE 2000
#define MIN_THROTTLE 1000

#define   redPin      6
#define   greenPin    A1
#define   bluePin     0

#define   CE          7
#define   CS          8

#define escPinFL      3
#define escPinFR      4
#define escPinRR      9
#define escPinRL      10

#define voltagePin    A6

#include <Wire.h>
#include <EEPROM.h>
#include <RF24.h>
#include <SPI.h>

// Time
long nextRefreshTime = refreshPeriod;

// Receiver
RF24 radio(CE, CS);
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

// Angles
float angleRoll, anglePitch, angleYaw;
bool firstAngle = false;

// ESCs
int escPulseFL, escPulseFR, escPulseRR, escPulseRL;

// State
bool start = false;
int state = 0;
int loopCounter = 0;
int motorTestOption = 0;

void setup()
{
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  pinMode(escPinFL, OUTPUT);
  pinMode(escPinFR, OUTPUT);
  pinMode(escPinRR, OUTPUT);
  pinMode(escPinRL, OUTPUT);

  radio.begin();
  radio.openReadingPipe(0, txAdress);
  radio.setPALevel(RF24_PA_MIN);      // one of four levels (MIN, LOW, HIGH, MAX)
  radio.setDataRate(RF24_2MBPS);      // or 250KBPS or 2Mbps
  radio.startListening();

  Wire.begin();
  TWBR = 12;                          // set I2C clock
  Serial.begin(115200);
  delay(250);

  Serial.print(">>> Starting calibration program");
  for (int i = 0; i < 3; i++) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();

  state = 0;
  loopCounter = 0;
}

void loop()
{
  if (state < 0) return;

  // wait for the refresh period to pass
  while (nextRefreshTime - micros() < refreshPeriod) { }
  nextRefreshTime = micros() + refreshPeriod;

  readReceiver();

  if (state == 0)
  {
    // ESC calibration, wait for minimum throttle
    if (loopCounter == 0)
    {
      Serial.println(">> ESC Calibration.");
      Serial.println("... Move throttle to maximum.");
      Serial.print("... Current throttle:\t");
      Serial.println(receiverThrottle);
    }
    else if (loopCounter % 125 == 0)
    {
      Serial.print("... Current throttle:\t");
      Serial.println(receiverThrottle);
    }
    loopCounter++;

    if (receiverThrottle > 1975)
    {
      Serial.print("... Maximum throttle detected: ");
      Serial.println(receiverThrottle);
      state = 1;
      loopCounter = 0;
    }
  }

  if (state == 1)
  {
    if (loopCounter == 0)
    {
      Serial.println("... Move throttle to minimum.");
      Serial.print("... Current throttle:\t");
      Serial.println(receiverThrottle);
    }
    else if (loopCounter % 125 == 0)
    {
      Serial.print("... Current throttle:\t");
      Serial.println(receiverThrottle);
    }

    // Throttle is maximum, send esc pulses
    if (receiverThrottle > 1975)
      escPulseOutput(receiverThrottle);

    if (loopCounter == 1125)
    {
      state = -1;
      Serial.println("!!! ESC calibraiton failed. Waited too long for minimum throttle.");
      Serial.println(">>> Reconnect the battery to restart the calibration program.");
    }

    loopCounter++;

    if (receiverThrottle < 1025)
    {
      Serial.print("... Minimum throttle detected: ");
      Serial.println(receiverThrottle);
      state = 2;
      loopCounter = 0;
    }
  }

  if (state == 2)
  {
    // Wait for ESC to takie the minimum throttle
    loopCounter++;
    escPulseOutput(receiverThrottle);

    if (loopCounter == 250)
    {
      Serial.println("... ESC calibration finished.");
      loopCounter = 0;
      state = 3;
    }
  }

  if (state == 3)
  {
    if (loopCounter == 0)
    {
      Serial.println(">>> Write following to test motors or finish testing:");
      Serial.println("... '1' to test front-left (CW) motor");
      Serial.println("... '2' to test front-right (CCW) motor");
      Serial.println("... '3' to test rear-right (CW) motor");
      Serial.println("... '4' to test rear-left (CCW) motor");
      Serial.println("... '5' to test all motors");
      Serial.println("... '6' to finish");
    }

    loopCounter++;
    if (Serial.available() > 0)
    {
      byte data = Serial.read();
      if (data == '1')
      {
        Serial.println("... Testing motor FL. Move throttle to test.");
        motorTestOption = 1;
      }
      else if (data == '2')
      {
        Serial.println("... Testing motor FR. Move throttle to test.");
        motorTestOption = 2;
      }
      else if (data == '3')
      {
        Serial.println("... Testing motor RR. Move throttle to test.");
        motorTestOption = 3;
      }
      else if (data == '4')
      {
        Serial.println("... Testing motor RL. Move throttle to test.");
        motorTestOption = 4;
      }
      else if (data == '5')
      {

        Serial.println("... Testing all motors. Move throttle to test.");
        motorTestOption = 5;
      }
      else if (data == '6')
      {
        state = 4;
        loopCounter = 0;
        motorTestOption = 0;
        Serial.println("... Testing motors done.");
      }
      while (Serial.available() > 0) data = Serial.read();
    }

    if (motorTestOption > 0 && loopCounter % 125 == 0)
    {
      Serial.print("... Current throttle:\t");
      Serial.println(receiverThrottle);
    }

    if (motorTestOption == 1)
      escPulseFL = receiverThrottle;
    else
      escPulseFL = 1000;
    if (motorTestOption == 2)
      escPulseFR = receiverThrottle;
    else escPulseFR = 1000;
    if (motorTestOption == 3)
      escPulseRR = receiverThrottle;
    else escPulseRR = 1000;
    if (motorTestOption == 4)
      escPulseRL = receiverThrottle;
    else
      escPulseRL = 1000;
    if (motorTestOption == 5)
    {
      escPulseOutput(receiverThrottle);
    }
    else
      escPulseOutput();
  }

  if (state == 4)
  {
    if (loopCounter == 0)
    {
      Serial.println(">>> Printing receiver signal.");
      Serial.println("... Move throttle minimum and yaw left to START or yaw right to STOP.");
    }
    loopCounter++;
    if (loopCounter > 500)
    {
      // Start condition
      if (!start && receiverThrottle < 1050 && receiverYaw < 1050) start = true;

      if (start)
      {
        // Stop condition
        if (start && receiverThrottle < 1050 && receiverYaw > 1950)
        {
          start = false;
          state = 5;
          loopCounter = 0;
        }
        printSignals();
      }
    }

    escPulseOutput(1000);
  }

  if (state == 5)
  {
    if (loopCounter == 0)
    {
      startGyro();
    }
    else if (loopCounter == 250)
      Serial.println(">>> Calibrating gyro in 3 seconds. Don't move the quadcopter!");
    else if (loopCounter < 1001 && loopCounter % 250 == 0)
      Serial.println(".");

    if (!gyroCalibrationDone)
    {
      loopCounter = 1001;
      for (int i = 0; i < 2000; i++)
      {
        if (i % 125 == 0)
        {
          digitalWrite(redPin, !digitalRead(redPin));   //Change the led status to indicate calibration.
          Serial.print(".");
        }
        readGyroSignals();                                                              //Read the gyro output.
        gyroRollOffset += gyroRoll;                                                     //Add roll value to gyro_roll_cal.
        gyroPitchOffset += gyroPitch;                                                   //Add pitch value to gyro_pitch_cal.
        gyroYawOffset += gyroYaw;                                                       //Add yaw value to gyro_yaw_cal.

        escPulseOutput(1000);
        delay(2);
      }

      gyroRollOffset /= 2000;                                                           //Divide the roll total by 2000.
      gyroPitchOffset /= 2000;                                                          //Divide the pitch total by 2000.
      gyroYawOffset /= 2000;                                                            //Divide the yaw total by 2000.
      gyroCalibrationDone = true;

      Serial.println();
      Serial.print("... Roll axis offset: ");
      Serial.println(gyroRollOffset);
      Serial.print("... Pitch axis offset: ");
      Serial.println(gyroPitchOffset);
      Serial.print("... Yaw axis offset: ");
      Serial.println(gyroYawOffset);
      Serial.println();
    }
    else
    {
      escPulseOutput(1000);
      readGyroSignals();
      calculateAngles();

      //We can't print all the data at once. This takes to long and the angular readings will be off.
      if (loopCounter == 1001)Serial.print("Pitch: ");
      if (loopCounter == 1002)Serial.print(anglePitch , 0);
      if (loopCounter == 1003)Serial.print(" Roll: ");
      if (loopCounter == 1004)Serial.print(angleRoll , 0);
      if (loopCounter == 1005)Serial.print(" Yaw: ");
      if (loopCounter == 1006)Serial.println(gyroYaw / 65.5 , 0);

      loopCounter++;
      if (loopCounter == 1060) loopCounter = 1001;
    }

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
  if (receiverThrottle < MIN_THROTTLE) receiverThrottle = MIN_THROTTLE;

  if (receiverYaw > 2000) receiverYaw = 2000;
  if (receiverPitch > 2000) receiverPitch = 2000;
  if (receiverRoll > 2000) receiverRoll = 2000;
  if (receiverThrottle > MAX_THROTTLE) receiverThrottle = MAX_THROTTLE;
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
  long timeStamp = micros();
  int pulsesDone = 0;
  PORTD |= B00011000;   // set D3 (PD3) (RL) and D4 (PD4) (RR) to high
  PORTB |= B00000110;   // set D9 (PB1) (FR) and D10 (PB2) (FL) to high
  long pulseOverRR = escPulseRR + timeStamp;
  long pulseOverRL = escPulseRL + timeStamp;
  long pulseOverFR = escPulseFR + timeStamp;
  long pulseOverFL = escPulseFL + timeStamp;

  while (pulsesDone < 4)
  {
    timeStamp = micros();
    if (timeStamp >= pulseOverRR)
    {
      PORTD &= B11101111;
      pulsesDone++;
    }
    if (timeStamp >= pulseOverRL)
    {
      PORTD &= B11110111;
      pulsesDone++;
    }
    if (timeStamp >= pulseOverFR)
    {
      PORTB &= B11111101;
      pulsesDone++;
    }
    if (timeStamp >= pulseOverFL)
    {
      PORTB &= B11111011;
      pulsesDone++;
    }
  }
}

void printSignals()
{
  Serial.print("Start:");
  Serial.print(start);

  Serial.print("\tRoll:");
  if (receiverRoll - 1480 < 0)Serial.print("<<<");
  else if (receiverRoll - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(receiverRoll);

  Serial.print("\tPitch:");
  if (receiverPitch - 1480 < 0)Serial.print("^^^");
  else if (receiverPitch - 1520 > 0)Serial.print("vvv");
  else Serial.print("-+-");
  Serial.print(receiverPitch);

  Serial.print("\tThrottle:");
  if (receiverThrottle - 1480 < 0)Serial.print("vvv");
  else if (receiverThrottle - 1520 > 0)Serial.print("^^^");
  else Serial.print("-+-");
  Serial.print(receiverThrottle);

  Serial.print("\tYaw:");
  if (receiverYaw - 1480 < 0)Serial.print("<<<");
  else if (receiverYaw - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.println(receiverYaw);
}

void startGyro()
{
  Serial.println(">>> Starting gyro.");

  Wire.beginTransmission(gyroAddress);                          //Start communication with the gyro
  Wire.write(0x6B);                                            //PWR_MGMT_1 register
  Wire.write(0x00);                                            //Set to zero to turn on the gyro
  Wire.endTransmission();                                      //End the transmission

  Wire.beginTransmission(gyroAddress);                         //Start communication with the gyro
  Wire.write(0x1B);                                            //GYRO_CONFIG register
  Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale)
  Wire.endTransmission();                                      //End the transmission

  Wire.beginTransmission(gyroAddress);                        //Start communication with the address found during search.
  Wire.write(0x1C);                                            //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x10);                                            //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission();                                      //End the transmission with the gyro

  Wire.beginTransmission(gyroAddress);                         //Start communication with the gyro (adress 1101001)
  Wire.write(0x1B);                                            //Start reading @ register 28h and auto increment with every read
  Wire.endTransmission();                                      //End the transmission
  Wire.requestFrom(gyroAddress, 1);                            //Request 1 bytes from the gyro
  while (Wire.available() < 1);                                //Wait until the 6 bytes are received
  if (Wire.read() != 0x08)
  { //Check if the value is 0x08
    digitalWrite(redPin, HIGH);                                //Turn on the warning led
    while (1) delay(10);                                        //Stay in this loop for ever
  }

  Wire.beginTransmission(gyroAddress);                        //Start communication with the address found during search
  Wire.write(0x1A);                                            //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                            //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                      //End the transmission with the gyro
}

void readGyroSignals()
{
  Wire.beginTransmission(gyroAddress);                         //Start communication with the gyro
  Wire.write(0x3B);                                            //Start reading @ register 43h and auto increment with every read
  Wire.endTransmission();                                      //End the transmission
  Wire.requestFrom(gyroAddress, 14);                           //Request 6 bytes from the gyro
  while (Wire.available() < 14);                               //Wait until the 6 bytes are received

  accX = Wire.read() << 8 | Wire.read();                       //Add the low and high byte to the acc_x variable.
  accY = Wire.read() << 8 | Wire.read();                       //Add the low and high byte to the acc_y variable.
  accZ = Wire.read() << 8 | Wire.read();
  byte temperature = Wire.read() << 8 | Wire.read();
  gyroPitch = Wire.read() << 8 | Wire.read();                  //Read high and low part of the angular data
  gyroRoll = Wire.read() << 8 | Wire.read();                   //Read high and low part of the angular data
  gyroYaw = Wire.read() << 8 | Wire.read();                    //Read high and low part of the angular data


  if (gyroCalibrationDone)                                     //Only compensate after the calibration
  {
    gyroRoll -= gyroRollOffset;
    gyroPitch -= gyroPitchOffset;
    gyroYaw -= gyroYawOffset;
  }
}

void calculateAngles()
{
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  anglePitch += gyroPitch * 0.0000611;
  angleRoll += gyroRoll * 0.0000611;

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  anglePitch -= angleRoll * sin(gyroYaw * 0.000001066);
  angleRoll += anglePitch * sin(gyroYaw * 0.000001066);

  //Accelerometer angle calculations
  accVector = sqrt((accX * accX) + (accY * accY) + (accZ * accZ));

  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  float anglePitchAcc = asin((float)accY / accVector) * 57.296;             //Calculate the pitch angle.
  float angleRollAcc = asin((float)accX / accVector) * -57.296;             //Calculate the roll angle.

  if (!firstAngle)
  {
    anglePitch = anglePitchAcc;                                                 //Set the pitch angle to the accelerometer angle.
    angleRoll = angleRollAcc;                                                   //Set the roll angle to the accelerometer angle.
    firstAngle = true;
  }
  else 
  {
    anglePitch = anglePitch * 0.9996 + anglePitchAcc * 0.0004;                 //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
    angleRoll = angleRoll * 0.9996 + angleRollAcc * 0.0004;                    //Correct the drift of the gyro roll angle with the accelerometer roll angle.
  }
}
