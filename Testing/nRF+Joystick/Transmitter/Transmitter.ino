//#define SHOWVALUES
//#define SHOWTIMES

#include <RF24.h>
#include <SPI.h> 

#define refreshPeriod  30000 // us
long nextRefreshTime = refreshPeriod;

#define SW_L  9   // digital input from left switch
#define X_L   A3  // analog input pin from left X
#define Y_L   A2  // analog input pin from left Y
#define SW_R  3   // digital pin connected to switch output
#define X_R   A5  // analog input pin from right X output
#define Y_R   A4  // analog input pin from right Y output

int data[4];     // data buffer

RF24 radio(7,8); // NRF pins - (CE, CS)
const byte address[6]="00001"; // only one-way, so address of receiver

void setup()
{
  #if defined(SHOWVALUES) || defined(SHOWTIMES)
    Serial.begin(115200);
  #endif
  
  //Joystick
  pinMode(SW_L, INPUT);
  pinMode(SW_R, INPUT);
  
  //nRF
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);  // one of four levels (MIN, LOW, HIGH, MAX)
  radio.setDataRate(RF24_2MBPS);  // or 250KBPS or 1MBPS
  radio.stopListening();          // because only transmitting
}

void loop() 
{

#ifdef SHOWTIMES

  // show how much time we need to wait until next loop
  long waitTime = nextRefreshTime - micros();
  Serial.print("Time to wait: "); 
  Serial.print(waitTime);
  Serial.print("\t");

#endif

  // wait for the refresh period to pass
  while (nextRefreshTime - micros() < refreshPeriod) { }
  nextRefreshTime = micros() + refreshPeriod;

#ifdef SHOWTIMES
  long startTime = micros();
#endif

  data[0] = analogRead(X_L);
  data[1] = analogRead(Y_L);
  data[2] = analogRead(X_R);
  data[3] = analogRead(Y_R);

#ifdef SHOWVALUES

  Serial.print("Yaw: ");
  Serial.print(data[0]);
  Serial.print("\t");
  
  Serial.print("Throttle: ");
  Serial.print(data[1]);
  Serial.print("\t");
  
  Serial.print("Roll: ");
  Serial.print(data[2]);
  Serial.print("\t");
  
  Serial.print("Pitch: ");
  Serial.print(data[3]);
  Serial.print("\t");

#endif

  radio.write(&data, sizeof(data), false);

#ifdef SHOWTIMES

  // show how much time we spent in the loop
  long totalTime = micros() - startTime;
  Serial.print("Total time: ");
  Serial.print(totalTime);
  
#endif

#if defined(SHOWVALUES) || defined(SHOWTIMES)
  Serial.println();
#endif
  
}
