// Arduino pin numbers
const int SW_pin_2 = 2; // digital pin connected to switch output
const int X_pin_0 = 0; // analog pin connected to X output
const int Y_pin_1 = 1; // analog pin connected to Y output

const int SW_pin_3 = 3; // digital pin connected to switch output
const int X_pin_2 = 2; // analog pin connected to X output
const int Y_pin_3 = 3; // analog pin connected to Y output

void setup() {
  pinMode(SW_pin_2, INPUT);
  digitalWrite(SW_pin_2, HIGH);
  pinMode(SW_pin_3, INPUT);
  digitalWrite(SW_pin_3, HIGH);
  Serial.begin(115200);
}

void loop() {
  Serial.print("Switch:  ");
  Serial.print(digitalRead(SW_pin_2));
  Serial.print("\n");
  Serial.print("1)X-axis: ");
  Serial.print(analogRead(X_pin_0));
  Serial.print("\n");
  Serial.print("2)Y-axis: ");
  Serial.println(analogRead(Y_pin_1));

  Serial.print("Switch:  ");
  Serial.print(digitalRead(SW_pin_3));
  Serial.print("\n");
  Serial.print("X-axis: ");
  Serial.print(analogRead(X_pin_2));
  Serial.print("\n");
  Serial.print("Y-axis: ");
  Serial.println(analogRead(Y_pin_3));
  
  Serial.print("\n\n");
  delay(100);
}
