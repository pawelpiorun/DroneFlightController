//do nadawania

#include <RF24.h>
#include <SPI.h> 

//nRF
RF24 radio(7,8); //piny, do których jest podpięty nrf (CE i CS)
int dane[4];

// Arduino pin numbers (Joystick)
const int SW_L = 9; // digital pin connected to switch output
const int X_L = 3; // analog pin connected to X output
const int Y_L = 2; // analog pin connected to Y output

const int SW_P = 3; // digital pin connected to switch output
const int X_P = 5; // analog pin connected to X output
const int Y_P = 4; // analog pin connected to Y output


const byte address[6]="00001"; //komunikacja tylko jednokierunkowa, wiec tylko adres nadajnika?
void setup() {
  Serial.begin(9600);
  //Joystick
  pinMode(SW_L, INPUT);
  digitalWrite(SW_L, HIGH);
  pinMode(SW_P, INPUT);
  digitalWrite(SW_P, HIGH);
  //nRF
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);//jeden z czterech poziomów mocy (MIN, LOW, HIGH, MAX)
  radio.setDataRate(RF24_2MBPS); // można też 1Mbps i 2Mbps
  radio.stopListening(); //bo tylko nadajemy
}

void loop() {
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
  
  radio.write(&dane, sizeof(dane));
  
  delay(40); //wysyłanie co 1s

}
