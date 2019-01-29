byte READPIN = A0;

void setup() {
  // put your setup code here, to run once:
  pinMode(READPIN, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(analogRead(READPIN));
}
