// IMPORTANT - READ THIS!
/*
 * - Disconnect the battery before starting this program.
 * - Initial pulse for ESC is 2000us to show maximum range value!!!
 * - After 3 beeps "ti da di" - ESC is initialized. 
 * Number of beeps may depend on battery voltage (in this case it's 3S).
 * - 1 short beep states that upper range value has been detected
 * - YOU HAVE 2 SECONDS TO WRITE "1" TO SERIAL PORT - it will change pulse to 1000us,
 * in order to show the minimum value to ESC.
 * - 1 short beep will state that lower range value has been detected.
 * - 1 long beep will be a message "Ready to fly"
 * - entering
 * 
 * Read SimonK ESC startup procedure for more info.
 * It is possible to initialize esc pulse with 1000us, but the upper value will not be detected.
 * SimonK ESC firmware should be working fine even without this procedure though.
 * As default is should have 1000us to 2000us range.
 */

#define escPin1 3

// TODO: Check for 4000 us
const float refreshPeriod = 20000; //us
long nextRefreshTime = 0;

float escPulse1 = 2000; //us

void setup() {
  pinMode(escPin1, OUTPUT);
  Serial.begin(9600);
}

void loop() {

  // wait until we hit next refresh time
  while (micros() < nextRefreshTime) { };

  // set next refresh time
  nextRefreshTime = micros() + refreshPeriod;

  // write PWM to esc
  digitalWrite(escPin1, HIGH);
  delayMicroseconds(escPulse1);
  digitalWrite(escPin1, LOW);

  // get command to change pulse
  // 1 - 1000 (no speed)
  // 2 - 1200
  // 3 - 1500
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

  // print current pulse
  Serial.println(escPulse1);
}
