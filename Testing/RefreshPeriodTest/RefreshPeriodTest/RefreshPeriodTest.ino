#define refreshPeriod 4000 // us

unsigned long timer;
unsigned long period;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  timer = micros();
}

void loop() {
  // put your main code here, to run repeatedly:

  // wait for the refresh period to pass
  while (micros() - timer < refreshPeriod) { }
  period = micros() - timer;
  timer = micros();

  Serial.println(period);
}
