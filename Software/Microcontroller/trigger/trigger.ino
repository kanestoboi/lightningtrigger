// Include libraries for LCD
#include <util/delay.h>
#include <Wire.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
#include <StateMachine.h>
#include <SoftwareSerial.h>// import the serial library
#include <ArduinoJson.h>
#include "Timelapse.h"
#include "ThresholdTrigger.h"
#include "BatteryIndicator.h"
#include "HDR.h"

void triggerCamera();
void focusCamera();
void releaseCamera();
void triggerFlash();

volatile bool BLUETOOTH_INTERRUPT_FLAG = false;

/*********************************
 * DEVELOPMENT SETUP
 ********************************/

int rx = 0;  // software serial RX pin
int tx = 1;  // software serial TX pin 
long counter = 0; // debugging counter
int statusLEDPin = 5;


/*********************************
 * CONFIGURATION SETUP
 ********************************/
/*int rx = 10;  // software serial RX pin
int tx = 11;  // software serial TX pin 
*/
#define OLED_RESET 4

#define F_CPU 16000000UL  // Define the microcontroller clock speed (16 MHz)


// Create various objects
//Adafruit_SSD1306 display(OLED_RESET);   // create LCD object
SoftwareSerial Bluetooth(rx, tx);       // create bluetooth object
Timelapse timelapse = Timelapse(&triggerCamera, &releaseCamera);
ThresholdTrigger thresholdTrigger = ThresholdTrigger(&focusCamera, &triggerCamera, &releaseCamera, &triggerFlash);
BatteryIndicator batteryIndicator = BatteryIndicator(7);
HDR hdr = HDR(&triggerCamera, &releaseCamera, &focusCamera);
StaticJsonDocument<150> bluetoothRxMessage;
DeserializationError error;

// Variables
int lightningSensitivity = 10;  // Threshold for light to trigger camera
int soundSensitivity = 10;


// Various Flags


// masks used
int focus1Mask = 0b00000010;        // PORT B Mask
int shutter1Mask = 0b00000001;      // PORT B Mask
int focus2Mask = 0b00001000;        // PORT B Mask
int shutter2Mask = 0b00000100;      // PORT B Mask
int focus3Mask = 0b00010000;        // PORT B Mask
int shutter3Mask = 0b00010000;      // PORT B Mask
int focus4Mask = 0b00001000;        // PORT C Mask
int shutter4Mask = 0b00010000;      // PORT C Mask
int cameraFocusMask = 0b00000001;   // PORT B Mask
int cameraTriggerMask = 0b00000010; // PORT B Mask
int flashTriggerMask = 0b00001100;  // PORT B Mask
int switchesMask = 0b01111100;
int downMask = 0b00000100;
int centerMask = 0b00001000;
int leftMask = 0b00010000;
int upMask = 0b00100000;
int rightMask = 0b01000000;

void setup() {
  //Serial.begin(38400);
  Bluetooth.begin(38400);  // Begin the bluetooth serial and set data rate
  Wire.begin();
  

  Bluetooth.println("Bluetooth connected");

  setSoundSensitivity(0);       // sets default sensitivity 
  setLightningSensitivity(127); // sets default sensitivity
  //display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  //display.clearDisplay();   // clear the adafruit splash screen
  
  /**********************************
   * THERE IS A BUG IN THE DISPLAY.BEGIN FUNCTION CAUSING PD4 TO BE SET HIGH
   * THE WORK AROUND BELOW SETS ALL PINS LOW BEFORE SETTING THEM AS INPUTS
   **********************************/
  DDRB = DDRB | 0b00111111; // set focus and shutter pins to be outputs
  //DDRD = 0b00000000;
  //PORTD = 0b00000000;
  //DDRD = DDRD | switchesMask; // set the 5-way switch pins to be inputs
  

//  display.setTextSize(1);
//  display.setTextColor(WHITE);
//  display.setCursor(0,0);
//  display.println("Lightning Trigger");
//  display.setCursor(0,20);
//  display.print("Threshold: ");
//  display.println(lightningThreshold);
//  display.display();


  attachInterrupt(0,bluetoothISR,  RISING); // setup interrupt for INT0 (UNO pin 2) //TODO: Change this to real interrupt
 


  //delay(4000);

  timer2InterruptSetup();

  pinMode(statusLEDPin, OUTPUT);

}



void loop() {
  //display.clearDisplay();
  //display.display();



  if (BLUETOOTH_INTERRUPT_FLAG) {
    readBluetoothString();
    Bluetooth.println("");
    Bluetooth.println("Bluetooth Message");
    //interrupts();
    if (error) {
      BLUETOOTH_INTERRUPT_FLAG = false;
      Bluetooth.println("DeserializationError");
    }
    else {
      thresholdTrigger.setSensitivity(bluetoothRxMessage["sensitivity"]);
      thresholdTrigger.setTriggerDelay(bluetoothRxMessage["delay"]);
      hdr.setCenterSpeed(bluetoothRxMessage["exposureCenter"]);
      hdr.setExposureValue(bluetoothRxMessage["exposureValue"]);

      cameraFocusMask = 0b00000000;
      cameraTriggerMask = 0b00000000;
      for (int i = 0; i < bluetoothRxMessage["cameraPort"].size(); i++)
      {
        if (bluetoothRxMessage["cameraPort"][i] == 1)
        {
          cameraFocusMask |= focus1Mask;
          cameraTriggerMask |= shutter1Mask;
        } else if (bluetoothRxMessage["cameraPort"][i] == 2) {
          cameraFocusMask |= focus2Mask;
          cameraTriggerMask |= shutter2Mask;
        } else if (bluetoothRxMessage["cameraPort"][i] == 3) {
          cameraFocusMask |= focus3Mask;
          cameraTriggerMask |= shutter3Mask;
        } else if (bluetoothRxMessage["cameraPort"][i] == 4) {
          cameraFocusMask |= focus4Mask;
          cameraTriggerMask |= shutter4Mask;
        }
      }

      flashTriggerMask = 0b00000000;
      for (int i = 0; i < bluetoothRxMessage["flashPort"].size(); i++)
      {
        if (bluetoothRxMessage["flashPort"][i] == 1)
        {
          flashTriggerMask |= focus1Mask;
          flashTriggerMask |= shutter1Mask;
        } else if (bluetoothRxMessage["flashPort"][i] == 2) {
          flashTriggerMask |= focus2Mask;
          flashTriggerMask |= shutter2Mask;
        } else if (bluetoothRxMessage["flashPort"][i] == 3) {
          flashTriggerMask |= focus3Mask;
          flashTriggerMask |= shutter3Mask;
        } else if (bluetoothRxMessage["flashPort"][i] == 4) {
          flashTriggerMask |= focus4Mask;
          flashTriggerMask |= shutter4Mask;
        }
      }

      if (bluetoothRxMessage["mode"] == "tm") {  // if the run time-lapse command was received from phone
        BLUETOOTH_INTERRUPT_FLAG = false;
        timelapseMode();
      }
      else if (bluetoothRxMessage["mode"] == "sm") {  // if the run sound mode command was received from phone
        BLUETOOTH_INTERRUPT_FLAG = false;
        soundMode();  
      }
      else if (bluetoothRxMessage["mode"] == "lm") {  // if the run lightning mode command was received from phone
        BLUETOOTH_INTERRUPT_FLAG = false;
        lightningMode();
      }
      else if (bluetoothRxMessage["mode"] == "hdr") {  // if the run HDR mode command was received from phone
        BLUETOOTH_INTERRUPT_FLAG = false;
        hdrMode();
      }
      else {
        BLUETOOTH_INTERRUPT_FLAG = false;
      }   
    }
  }

  //timelapseMode();
  
  //lightningMode();  
  
}

void bluetoothISR(){
  BLUETOOTH_INTERRUPT_FLAG = true; // trigger flag indicating a bluetoothRxMessage was received 
  //bluetoothRxMessage = Bluetooth.read();  // read the messgage
}

void setSoundSensitivity(int val) {
  Wire.beginTransmission(0x2C); // transmit to device 
  // device address is specified in datasheet
  Wire.write(byte(0x00));            // sends instruction byte
  Wire.write(val);             // sends potentiometer value byte
  Wire.endTransmission();     // stop transmitting
}

void setLightningSensitivity(int val) {
  Wire.beginTransmission(0x2E); // transmit to device 
  // device address is specified in datasheet
  Wire.write(byte(0x00));            // sends instruction byte
  Wire.write(val);             // sends potentiometer value byte
  Wire.endTransmission();     // stop transmitting
}

void setupSoundMode() {
  thresholdTrigger.setADCInput(1);
  thresholdTrigger.setup();
  thresholdTrigger.reset();
  
}

void setupLightningMode() {
  thresholdTrigger.setADCInput(0); // Set multiplexer to 0
  thresholdTrigger.setup();
  thresholdTrigger.reset();
}

static void focusCamera() {
  PORTB |= cameraFocusMask;  // focus the camera  
}

static void triggerCamera() {
  PORTB |= cameraTriggerMask;  // trigger the outputs  
}

static void releaseCamera() {
  PORTB = 0b00000000;   // reset trigger outputs to off
  PORTC &= 0b11100111;
}

void triggerFlash() {
  PORTB |= flashTriggerMask;  // trigger the outputs
  _delay_ms(200);
  PORTB = 0b00000000;   // reset trigger outputs to off
}


void updateDisplay() {
//  display.clearDisplay();
//  display.setCursor(0,20);
//  display.println(thresholdTrigger.getNumberOfTriggers());
//  display.display();
}

void lightningMode() {
//  display.clearDisplay();
//  display.display();
//  display.setCursor(0,0);
//  display.println("Lightning Trigger Mode");
//  display.display();
  
  setupLightningMode();
  Bluetooth.println("Lightning Mode");
  BLUETOOTH_INTERRUPT_FLAG = false; //TODO: need to find out why bluetooth flag is being triggered from above line

  thresholdTrigger.setSensitivity(bluetoothRxMessage["sensitivity"]); 

  thresholdTrigger.resetCalibration();
  ADCSRA |= B01000000;
  while(!thresholdTrigger.isCalibrated())
    thresholdTrigger.calibrateThreshold();

  Bluetooth.println("Calibrated  Lightning Mode");

//  display.clearDisplay();
//  display.display();
//  display.setCursor(0,0);
//  display.println("Lightning Trigger Running");
//  display.display();
//  Serial.println("calibration done");
  ADCSRA |= B01000000;
  thresholdTrigger.focusCamera();

  int triggers = 0;
  while(1) {
    if (triggers < thresholdTrigger.getNumberOfTriggers()){
      Bluetooth.print("Number of Triggers: ");
      Bluetooth.println(thresholdTrigger.getNumberOfTriggers());
      triggers = thresholdTrigger.getNumberOfTriggers();
    }
    thresholdTrigger.run();
    if (BLUETOOTH_INTERRUPT_FLAG) {
      thresholdTrigger.end();
      readBluetoothString();
      Bluetooth.println("Exiting Lightning Mode");
      return;
    }
  }
}

void soundMode() {
  setupSoundMode();
  Bluetooth.println("Sound Mode");

  //while(1) {

    thresholdTrigger.resetCalibration();
    ADCSRA |= B01000000;
    while(!thresholdTrigger.isCalibrated())
      thresholdTrigger.calibrateThreshold();
  
    Bluetooth.println("Waiting for Sound");
    thresholdTrigger.triggerCamera();
    delay(1000);
    
    BLUETOOTH_INTERRUPT_FLAG = false;
    while(!thresholdTrigger.run()) {
      if (BLUETOOTH_INTERRUPT_FLAG) {
        thresholdTrigger.end();
        readBluetoothString();
        Bluetooth.println("Exiting Sound Mode");
        return;
                
      }
  //    Bluetooth.print(THRESHOLD_TRIGGER_THRESHOLD);
  //    Bluetooth.print(" | ");
  //    Bluetooth.write(thresholdTrigger.analogVal());
    }
    thresholdTrigger.end();
    delay(1000);
    Bluetooth.println("Sound Triggered");

  //}
  
}

void timelapseMode() {
  
  Bluetooth.write("Timelapse Mode");
  BLUETOOTH_INTERRUPT_FLAG = false;   //TODO: need to find out why bluetooth flag is being triggered from above line
  timelapse.reset();

  timelapse.setExposure(bluetoothRxMessage["exposure"]);
  timelapse.setTotalPhotos(bluetoothRxMessage["photos"]);
  timelapse.setDelayBetweenShots(bluetoothRxMessage["delay"]);
  
  
  while (!timelapse.isDone()) {
    timelapse.run();
    if (BLUETOOTH_INTERRUPT_FLAG) {
      readBluetoothString();
      if (bluetoothRxMessage["flag"] == "err") {  // if the eit time-lapse command was received from phone
        timelapse.end();
        BLUETOOTH_INTERRUPT_FLAG = false;
        break;
      }
      BLUETOOTH_INTERRUPT_FLAG = false; // reset interrupt flag
    }
  }
  
  if (timelapse.isDone())
    Bluetooth.write("Complete");
  else 
    Bluetooth.write("Exited");
  
}

void hdrMode() {
  Bluetooth.println("HDR Mode");
  BLUETOOTH_INTERRUPT_FLAG = false;

  hdr.setPhotoTimeExposures();

  Bluetooth.println("Running HDR");

  hdr.reset();

  delay(2000);
  while (!hdr.isDone()) {
    hdr.run();
  }
  Bluetooth.println("HDR Done");
}

void readBluetoothString() {
  // disable timer compare interrupt
  TIMSK1 |= (0 << OCIE1A);
  String incommingMessage = ""; //clears variable for new input
  char c = ' ';
  while (c != '}') {
    // TODO: Add timeout timer for this
    if(Bluetooth.available()) {
      c = Bluetooth.read();  //gets one byte from serial buffer
      incommingMessage += c; 

      if (c == '}') {
        //Serial.println("Breaking");
        break;
      }  //breaks out of capture loop to print readstring
    } //makes the string readString  
  }


  if (incommingMessage.length() >0) {
    //Serial.println(incommingMessage); 
    //Bluetooth.println(incommingMessage); // sends string back to phone for verifiation

    error = deserializeJson(bluetoothRxMessage, incommingMessage);

    if (error) {
      //Bluetooth.write(F("deserializeJson() failed: "));
      //Bluetooth.println(error.c_str());
      return;
    }
  }
}

boolean isValidNumber(String str){
  for(byte i=0;i<str.length();i++)
  {
    if(isDigit(str.charAt(i))) 
      return true;
  }
   return false;
} 

void timer2InterruptSetup() {
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 65535;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
}

/**********************************
 * OVERFLOW TIMER RUNS "ASYNCHRONOUS" CODE EVERY 4 SECONDS
 **********************************/
ISR(TIMER1_COMPA_vect) { //timer1 interrupt 4Hz 
  

  int batteryLevel = batteryIndicator.getBatteryLevelPercentage();
  Bluetooth.print("Battery Level %: ");
  Bluetooth.print(batteryLevel);
  Bluetooth.println("%");

  Bluetooth.print("batteryStatus: ");
  Bluetooth.print(batteryIndicator.getBatteryStatus());
  Bluetooth.println("");
  //setupLightningMode();*/
  
}
