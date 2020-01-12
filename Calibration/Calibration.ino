#define   redPin      6
#define   greenPin    A1
#define   bluePin     0

#define   CE          7
#define   CS          8

#include <Wire.h>
#include <EEPROM.h>
#include <RF24.h>
#include <SPI.h>

const byte txAdress[6] = "00001";      // only one-way communication

RF24 radio(CE, CS);
bool error = false;
int receiverYaw, receiverThrottle, receiverPitch, receiverRoll;
int receiverData[4];

byte gyroAddress = 0x68;
float gyroRoll, gyroPitch, gyroYaw;
float gyroRollOffset, gyroPitchOffset, gyroYawOffset;
bool gyroCalibrationDone = false;

void setup()
{
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  radio.begin();
  radio.openReadingPipe(0, txAdress);
  radio.setPALevel(RF24_PA_MIN);      //one of four levels (MIN, LOW, HIGH, MAX)
  radio.setDataRate(RF24_2MBPS);      // or 250KBPS or 2Mbps
  //radio.startListening();

  Wire.begin();
  Serial.begin(115200);
  delay(250);
}

void loop()
{
  if (error) while (1) {
      delay(3000);
      Serial.print(".");
    }

  Serial.println("Flight controller calibration started.");

  if (checkI2C()) return;
  if (checkReceiver()) return;

  // TODO? Check sticks center position
  // TODO? Check sticks movement, assing pins, check directions, reversals, etc.
  // TODO? Check sticks boundries

  if (checkGyro()) return;
  startGyro();
  calibrateGyro();
  if (checkGyroAxes()) return;

  // TODO? Store info to EEPROM
  // TODO? Check LEDs

  Serial.println(">>> Setup is finished.");
  while (1);
}

bool checkI2C()
{
  Serial.print(">>> Checking I2C clock speed.");
  delay(1000);

  TWBR = 12;                      //Set the I2C clock speed to 400kHz.

  bool clockFine = false;
#if F_CPU == 16000000L
  clockFine = true;
#endif

  if (TWBR == 12 && clockFine)
    Serial.println("... 400kHz. OK!");
  else
  {
    Serial.println("... Not set to 400kHz. ERROR!!!!!!!");
    error = true;
  }

  delay(2000);
  return error;
}

bool checkReceiver()
{
  Serial.println(">>> Checking receiver.");
  radio.startListening();
  delay(100);

  bool dataArrived = false;
  unsigned long timer = millis() + 10000;
  while (timer > millis()) {
    if (radio.available())
    {
      Serial.println("... Data arrived.");
      dataArrived = true;
      readReceiver();
      error = checkReceiverData();
      break;
    }
  }

  if (!dataArrived)
  {
    Serial.println("... Data did not arrived! ERROR!!!!!");
    error = true;
  }
  Serial.print("Throttle: "); Serial.print(receiverThrottle);
  Serial.print("Yaw: "); Serial.print(receiverYaw);
  Serial.print("Pitch: "); Serial.print(receiverPitch);
  Serial.print("Roll: "); Serial.print(receiverRoll);

  if (error)
    Serial.println("... Corrupted receiver data. ERROR!!!!!!");
  else
    Serial.println("... Data received properly. OK!");

  delay(2000);
  return error;
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
  }
}

bool checkReceiverData()
{
  error = receiverYaw < 1000 || receiverYaw > 2000
          || receiverThrottle < 1000 || receiverThrottle > 2000
          || receiverRoll < 1000 || receiverRoll > 2000
          || receiverPitch < 1000 || receiverPitch > 2000;
  return error;
}

bool checkGyro()
{
  bool found = false;

  gyroAddress = 0x68;
  byte lowByte;
  Serial.println(">>> Checking gyro.");
  Serial.println("... Searching for MPU-6050 on address 0x68/104");
  Wire.beginTransmission(gyroAddress);
  Wire.write(0x75);
  Wire.endTransmission();
  Wire.requestFrom(gyroAddress, 1);
  delay(100);
  lowByte = Wire.read();
  if (lowByte == 0x68) found = true;

  delay(2000);

  if (!found) {
    gyroAddress = 0x69;
    Serial.println("... Searching for MPU-6050 on address 0x69/105");
    delay(1000);
    Wire.beginTransmission(gyroAddress);
    Wire.write(0x75);
    Wire.endTransmission();
    Wire.requestFrom(gyroAddress, 1);
    delay(100);
    lowByte = Wire.read();
    if (lowByte == 0x68) found = true;
  }

  delay(2000);

  if (!found)
    Serial.println("... No gyro device found!!! ERROR!!!");
  else
    Serial.println("... MPU-6050 gyro found! Showing register settings...");

  delay(2000);

  error = !found;
  return error;
}

void startGyro()
{
  Serial.println(">>> Starting gyro.");
  
  Wire.beginTransmission(gyroAddress);                             //Start communication with the gyro
  Wire.write(0x6B);                                            //PWR_MGMT_1 register
  Wire.write(0x00);                                            //Set to zero to turn on the gyro
  Wire.endTransmission();                                      //End the transmission

  Wire.beginTransmission(gyroAddress);                             //Start communication with the gyro
  Wire.write(0x6B);                                            //Start reading @ register 28h and auto increment with every read
  Wire.endTransmission();                                      //End the transmission
  Wire.requestFrom(gyroAddress, 1);                                //Request 1 bytes from the gyro
  while (Wire.available() < 1);                                //Wait until the 1 byte is received
  Serial.print("... Register 0x6B is set to:");
  Serial.println(Wire.read(), BIN);

  Wire.beginTransmission(gyroAddress);                             //Start communication with the gyro
  Wire.write(0x1B);                                            //GYRO_CONFIG register
  Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale)
  Wire.endTransmission();                                      //End the transmission

  Wire.beginTransmission(gyroAddress);                             //Start communication with the gyro (adress 1101001)
  Wire.write(0x1B);                                            //Start reading @ register 28h and auto increment with every read
  Wire.endTransmission();                                      //End the transmission
  Wire.requestFrom(gyroAddress, 1);                                //Request 1 bytes from the gyro
  while (Wire.available() < 1);                                //Wait until the 1 byte is received
  Serial.print("... Register 0x1B is set to:");
  Serial.println(Wire.read(), BIN);

  delay(2000);
}

void calibrateGyro()
{
  Serial.println(">>> Calibrating gyro will start in 3 seconds. DONT MOVE THE QUADCOPTER!!!");
  Serial.println("."); delay(1000); Serial.println("."); delay(1000); Serial.println("."); delay(1000);
  Serial.println(">>> Calibrating gyro, please wait...");

  for (int i = 0; i < 2000 ; i++)
  {
    if (i % 100 == 0)Serial.print(".");                        //Print dot to indicate calibration.
    readGyroSignals();                                         //Read the gyro output.
    gyroRollOffset += gyroRoll;                                //Ad roll value to gyro_roll_cal.
    gyroPitchOffset += gyroPitch;                              //Ad pitch value to gyro_pitch_cal.
    gyroYawOffset += gyroYaw;                                  //Ad yaw value to gyro_yaw_cal.
    delay(4);                                                  //Wait 3 milliseconds before the next loop.
  }
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyroRollOffset /= 2000;                                       //Divide the roll total by 2000.
  gyroPitchOffset /= 2000;                                      //Divide the pitch total by 2000.
  gyroYawOffset /= 2000;                                        //Divide the yaw total by 2000.

  Serial.println();
  Serial.print("... Axis 1 offset=");
  Serial.println(gyroRollOffset);
  Serial.print("... Axis 2 offset=");
  Serial.println(gyroPitchOffset);
  Serial.print("... Axis 3 offset=");
  Serial.println(gyroYawOffset);
  Serial.println();
  delay(2000);
}

void readGyroSignals()
{
  Wire.beginTransmission(gyroAddress);                             //Start communication with the gyro
  Wire.write(0x43);                                            //Start reading @ register 43h and auto increment with every read
  Wire.endTransmission();                                      //End the transmission
  Wire.requestFrom(gyroAddress, 6);                                //Request 6 bytes from the gyro
  while (Wire.available() < 6);                                //Wait until the 6 bytes are received

  gyroRoll = Wire.read() << 8 | Wire.read();                   //Read high and low part of the angular data
  gyroPitch = Wire.read() << 8 | Wire.read();                  //Read high and low part of the angular data
  gyroYaw = Wire.read() << 8 | Wire.read();                    //Read high and low part of the angular data

  if (gyroCalibrationDone)                                     //Only compensate after the calibration
  {
    gyroYaw -= gyroYawOffset;
    gyroRoll -= gyroRollOffset;
    gyroPitch -= gyroPitchOffset;
  }
}

bool checkGyroAxes()
{
  error = checkGyroAxis(1) || checkGyroAxis(2) || checkGyroAxis(3);
  return error;
}

bool checkGyroAxis(int axis)
{
  if (axis == 1)
  {
    Serial.println(">>> Checking roll axis.");
    Serial.println(">>> Please lift left side to 45 degrees.");
  }
  else if (axis == 2)
  {
    Serial.println(">>> Checking pitch axis.");
    Serial.println(">>> Please rotate nose up to 45 degrees.");
  }
  else if (axis == 3)
  {
    Serial.println(">>> Checking yaw axis.");
    Serial.println(">>> Please rotate nose right to 45 degrees.");
  }

  int triggeredAxis = 0;
  float gyroRollAngle = 0, gyroPitchAngle = 0, gyroYawAngle = 0;

  unsigned long timer = millis() + 10000;
  while (timer > millis() && abs(gyroRollAngle) < 30 && abs(gyroPitchAngle) < 30 && abs(gyroYawAngle) < 30)
  {
    readGyroSignals();
    gyroRollAngle += gyroRoll * 0.0000611;          // 0.0000611 = 1 / 65.5 (LSB degr/s) / 250(Hz)
    gyroPitchAngle += gyroPitch * 0.0000611;
    gyroYawAngle += gyroYaw * 0.0000611;
    delayMicroseconds(3700);
  }

  if (abs(gyroRollAngle) > 30)
  {
    if (axis == 1)
      Serial.println("... Roll axis OK!");
    else
    {
      Serial.println("... Wrong axis!!! Detected as roll axis.");
      error = true;
    }
    triggeredAxis = 1;
  }
  else if (abs(gyroPitchAngle) > 30)
  {
    if (axis == 2)
      Serial.println("... Pitch axis OK!");
    else
    {
      Serial.println("... Wrong axis!!! Detected as pitch axis.");
      error = true;
    }
    triggeredAxis = 2;
  }
  else if (abs(gyroYawAngle) > 30)
  {
    if (axis == 3)
      Serial.println("... Yaw axis OK!");
    else
    {
      Serial.println("... Wrong axis!!! Detected as yaw axis.");
      error = true;
    }
    triggeredAxis = 3;
  }

  if (triggeredAxis == 0)
  {
    Serial.println("... No axis detected! ERROR!");
    error = true;
  }

  delay(2000);
  return error;
}
