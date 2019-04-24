#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); // RX, TX

void setup() {

Serial.begin(9600);

pinMode(9,OUTPUT); digitalWrite(9,HIGH);

Serial.println("Enter AT commands:");

mySerial.begin(115200);

}

void loop()

{

if (mySerial.available())

Serial.write(mySerial.read());

if (Serial.available())

mySerial.write(Serial.read());

}
