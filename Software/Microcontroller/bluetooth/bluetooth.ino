#include <SoftwareSerial.h>// import the serial library

int rx = 10;  // software serial RX pin
int tx = 11;  // software serial TX pin
volatile bool interruptFlag = false;
volatile char message;

SoftwareSerial Bluetooth(10, 11); // RX, TX

void setup() {
  Bluetooth.begin(9600);  // Begin the bluetooth serial and set data rate
  Bluetooth.println("Bluetooth connected");

  Serial.begin(9600);

  attachInterrupt(0,bluetoothISR,  RISING); // setup interrupt for INT0 (UNO pin 2)
}

void loop() {
  // if the ISR has been triggered print the message to the hardware serial
  if (interruptFlag) {
    Serial.println(message);
    interruptFlag = false;  // clear the interrupt trigger flag
  }
}

void bluetoothISR(){
  interruptFlag = true; // trigger flag indicating a message was received 
  message = Bluetooth.read();  // read the messgage
}
