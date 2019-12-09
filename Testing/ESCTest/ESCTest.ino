#define escPin1 3

const float refreshPeriod = 20000; //us
long refreshStartTime = 0;

float escPulse1 = 1000; //us

void setup() {
  pinMode(escPin1, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  while (micros() < refreshStartTime) { };
  refreshStartTime = micros() + refreshPeriod;
  digitalWrite(escPin1, HIGH);
  delayMicroseconds(escPulse1);
  digitalWrite(escPin1, LOW);
  if (Serial.available() > 0)
  {
    char input = Serial.read();
    if (input == '1')
    {
      escPulse1 = 1000;
      Serial.println("Pulse changed to 1000");
    }
    else if (input == '2')
    {
      escPulse1 = 1200;
      Serial.println("Pulse changed to 1200");
    }
    else if (input == '3')
    {
      escPulse1 = 1500;
      Serial.println("Pulse changed to 1500");
    }
  }  
}
