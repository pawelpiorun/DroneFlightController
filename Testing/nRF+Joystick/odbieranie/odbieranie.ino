//do odbierania

#include <RF24.h>
#include <SPI.h> 

RF24 radio (7,8);//piny CE i CS

const byte address[6]="00001"; //komunikacja tylko jednokierunkowa, wiec tylko adres nadajnika?
void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0,address);
  radio.setPALevel(RF24_PA_MIN);//jeden z czterech poziomów mocy (MIN, LOW, HIGH, MAX)
  radio.setDataRate(RF24_2MBPS); // można też 1Mbps i 2Mbps
  radio.startListening(); //tutaj odbieramy
}

void loop() {
  
  
if(radio.available())
{
  int dane[4];
  radio.read(&dane, sizeof(dane));

  Serial.print("Yaw: ");
  Serial.print(dane[0]);
  Serial.print("\t");
  Serial.print("Throttle/*: ");
  Serial.print(dane[1]);
  Serial.print("\t");
  Serial.print("Roll: ");
  Serial.print(dane[2]);
  Serial.print("\t");
  Serial.print("Pitch: ");
  Serial.print(dane[3]);
  Serial.print("\n");
}

//delay(250);
}
