#define SHOWVALUES
//#define SHOWTIMES

#include <RF24.h>
#include <SPI.h> 

RF24 radio (7,8); // NRF pins (CE, CS)

const byte address[6]="00001"; // only one-way communication

void setup() 
{

#if defined(SHOWVALUES) || defined(SHOWTIMES)

  Serial.begin(115200);

#endif

  radio.begin();
  radio.openReadingPipe(0,address);
  radio.setPALevel(RF24_PA_MIN);    //one of four levels (MIN, LOW, HIGH, MAX)
  radio.setDataRate(RF24_2MBPS);    // or 250KBPS or 2Mbps
  radio.startListening();       
}

long lastData = 0;
void loop() 
{
    
  if(radio.available())
  {

#ifdef SHOWTIMES

    long timeFromLast = micros() - lastData;
    Serial.print("Time period: ");
    Serial.print(timeFromLast);
    Serial.print("\t");

    lastData = micros();
    long startTime = micros();

#endif
    
    int data[4];
    radio.read(&data, sizeof(data));

#ifdef SHOWVALUES

    Serial.print("Yaw: ");
    Serial.print(data[0]);
    Serial.print("\t\t");
    Serial.print("Throttle: ");
    Serial.print(data[1]);
    Serial.print("\t\t");
    Serial.print("Roll: ");
    Serial.print(data[2]);
    Serial.print("\t\t");
    Serial.print("Pitch: ");
    Serial.print(data[3]);
    Serial.print("\t\t"); 
      
#endif

#ifdef SHOWTIMES

    long totalTime = micros() - startTime;
    Serial.print("Total time: ");
    Serial.print(totalTime);

#endif

#if defined(SHOWVALUES) || defined(SHOWTIMES)
  
    Serial.println();

#endif

  }

}
