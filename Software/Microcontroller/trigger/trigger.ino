// Include libraries for LCD
#include <util/delay.h>


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

volatile bool Serial_INTERRUPT_FLAG = false;

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
//SoftwareSerial Serial(rx, tx);       // create Serial object
Timelapse timelapse = Timelapse(&triggerCamera, &releaseCamera);
ThresholdTrigger thresholdTrigger = ThresholdTrigger(&focusCamera, &triggerCamera, &releaseCamera, &triggerFlash);
BatteryIndicator batteryIndicator = BatteryIndicator(7);
HDR hdr = HDR(&triggerCamera, &releaseCamera, &focusCamera);
StaticJsonDocument<150> SerialRxMessage;
StaticJsonDocument<150> SerialTxMessage;
DeserializationError error;

// Variables
int lightningSensitivity = 10;  // Threshold for light to trigger camera
int soundSensitivity = 10;
const char* currentMode = "";


// Various Flags


// masks used
int focus1Mask = 0b00000010;        // PORT B Mask
int shutter1Mask = 0b00000001;      // PORT B Mask
int focus2Mask = 0b00001000;        // PORT B Mask
int shutter2Mask = 0b00000100;      // PORT B Mask
int focus3Mask = 0b00100000;        // PORT B Mask
int shutter3Mask = 0b00010000;      // PORT B Mask
int focus4Mask = 0b00001000;        // PORT D Mask
int shutter4Mask = 0b00010000;      // PORT D Mask
int PORTBCameraFocusMask = 0b00000000;   // PORT B Mask
int PORTBCameraTriggerMask = 0b00000000; // PORT B Mask
int PORTBFlashTriggerMask = 0b00000000;  // PORT B Mask

int PORTDCameraFocusMask = 0b00000000;   // PORT B Mask
int PORTDCameraTriggerMask = 0b00000000; // PORT B Mask
int PORTDFlashTriggerMask = 0b00000000;  // PORT B Mask


void setup() {

  char c;
  //Serial.begin(38400);
  Serial.begin(115200);  // Begin the Serial serial and set data rate

  
/*
  Serial.print("A");
    delay(100);
    while(Serial.available())
            c = Serial.read();

    Serial.print("AT");
            delay(100);

    while(Serial.available())
            c = Serial.read();


    delay(100);

    Serial.print("AT+NAMEDSLR REMOTE");
    delay(100);
        while(Serial.available())
            c = Serial.read();

    delay(100);

    Serial.print("AT+MODE2");
    delay(100);
        while(Serial.available())
            c = Serial.read();

*/

            

            
  Serial.println("Serial connected");
  sei();

  
  /**********************************
   * THERE IS A BUG IN THE DISPLAY.BEGIN FUNCTION CAUSING PD4 TO BE SET HIGH
   * THE WORK AROUND BELOW SETS ALL PINS LOW BEFORE SETTING THEM AS INPUTS
   **********************************/
  DDRB = DDRB | 0b00111111; // set focus and shutter pins to be outputs
  DDRD = DDRD | 0b00110000;

  attachInterrupt(0,SerialISR,  RISING); // setup interrupt for INT0 (UNO pin 2) //TODO: Change this to real interrupt

  timer2InterruptSetup();

  pinMode(statusLEDPin, OUTPUT);
  digitalWrite(statusLEDPin, LOW);


}


void loop() {

  if (Serial_INTERRUPT_FLAG) {
    readSerialString();
    if (error) {
      Serial_INTERRUPT_FLAG = false;
      Serial.println("DeserializationError");
    }
    else {
      thresholdTrigger.setSensitivity(SerialRxMessage["sensitivity"]);
      thresholdTrigger.setTriggerDelay(SerialRxMessage["delay"]);
      hdr.setCenterSpeed(SerialRxMessage["exposureCenter"]);
      hdr.setExposureValue(SerialRxMessage["exposureValue"]);

      PORTBCameraFocusMask = 0b00000000;
      PORTBCameraTriggerMask = 0b00000000;
      for (int i = 0; i < SerialRxMessage["cameraPort"].size(); i++)
      {
        if (SerialRxMessage["cameraPort"][i] == 1)
        {
          PORTBCameraFocusMask |= focus1Mask;
          PORTBCameraTriggerMask |= shutter1Mask;
          
        } else if (SerialRxMessage["cameraPort"][i] == 2) {
          PORTBCameraFocusMask |= focus2Mask;
          PORTBCameraTriggerMask |= shutter2Mask;
        } else if (SerialRxMessage["cameraPort"][i] == 3) {
          PORTBCameraFocusMask |= focus3Mask;
          PORTBCameraTriggerMask |= shutter3Mask;
        } else if (SerialRxMessage["cameraPort"][i] == 4) {
          PORTDCameraFocusMask |= focus4Mask;
          PORTDCameraTriggerMask |= shutter4Mask;
        }
      }

      PORTBFlashTriggerMask = 0b00000000;
      PORTDFlashTriggerMask &= 0b11001111;
      for (int i = 0; i < SerialRxMessage["flashPort"].size(); i++)
      {
        if (SerialRxMessage["flashPort"][i] == 1)
        {
          PORTBFlashTriggerMask |= focus1Mask;
          PORTBFlashTriggerMask |= shutter1Mask;
        } else if (SerialRxMessage["flashPort"][i] == 2) {
          PORTBFlashTriggerMask |= focus2Mask;
          PORTBFlashTriggerMask |= shutter2Mask;
        } else if (SerialRxMessage["flashPort"][i] == 3) {
          PORTBFlashTriggerMask |= focus3Mask;
          PORTBFlashTriggerMask |= shutter3Mask;
        } else if (SerialRxMessage["flashPort"][i] == 4) {
          PORTDFlashTriggerMask |= focus4Mask;
          PORTDFlashTriggerMask |= shutter4Mask;
        }
      }

      currentMode = SerialRxMessage["mode"];
      sendAsyncInfo();

      if (SerialRxMessage["mode"] == "tm") {  // if the run time-lapse command was received from phone
        Serial_INTERRUPT_FLAG = false;
        digitalWrite(statusLEDPin, HIGH);
        timelapseMode();
      }
      else if (SerialRxMessage["mode"] == "sm") {  // if the run sound mode command was received from phone
        Serial_INTERRUPT_FLAG = false;
        digitalWrite(statusLEDPin, HIGH);
        soundMode();  
      }
      else if (SerialRxMessage["mode"] == "lm") {  // if the run lightning mode command was received from phone
        Serial_INTERRUPT_FLAG = false;
        digitalWrite(statusLEDPin, HIGH);
        lightningMode();
      }
      else if (SerialRxMessage["mode"] == "hdr") {  // if the run HDR mode command was received from phone
        Serial_INTERRUPT_FLAG = false;
        digitalWrite(statusLEDPin, HIGH);
        hdrMode();
      }
      else if (SerialRxMessage["mode"] == "ss") {  // if the run single shot mode command was received from phone
        Serial_INTERRUPT_FLAG = false;
        digitalWrite(statusLEDPin, HIGH);
        singleShotMode();        
      }
      else {
        Serial_INTERRUPT_FLAG = false;
      }

      digitalWrite(statusLEDPin, LOW);

      currentMode = "";   
    }
  }  
}

void SerialISR(){
  Serial_INTERRUPT_FLAG = true; // trigger flag indicating a SerialRxMessage was received 
  digitalWrite(statusLEDPin, HIGH);
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
  PORTB |= PORTBCameraFocusMask;  // focus the camera  
  PORTD |= PORTDCameraFocusMask;  // focus the camera
}

static void triggerCamera() {
  PORTB |= PORTBCameraTriggerMask;  // trigger the outputs
  PORTD |= PORTDCameraTriggerMask;  
}

static void releaseCamera() {
  PORTB &= ~(PORTBCameraFocusMask | PORTBCameraTriggerMask);   // reset camera outputs to off
  PORTD &= ~(PORTDCameraFocusMask | PORTDCameraTriggerMask);
}

void triggerFlash() {
  PORTB |= PORTBFlashTriggerMask;  // trigger the outputs
  PORTD |= PORTDFlashTriggerMask;  // trigger the outputs
  _delay_ms(200);
  PORTB &= ~PORTBFlashTriggerMask;   // reset trigger outputs to off
  PORTD &= ~PORTDFlashTriggerMask;   // reset trigger outputs to off
}

void lightningMode() {  
  setupLightningMode();
  Serial.println("Lightning Mode");
  Serial_INTERRUPT_FLAG = false; //TODO: need to find out why Serial flag is being triggered from above line

  thresholdTrigger.setSensitivity(SerialRxMessage["sensitivity"]); 

  thresholdTrigger.resetCalibration();
  ADCSRA |= B01000000;
  while(!thresholdTrigger.isCalibrated())
    thresholdTrigger.calibrateThreshold();

  Serial.println("Calibrated  Lightning Mode");

  ADCSRA |= B01000000;
  thresholdTrigger.focusCamera();
  counter = 0;
  int triggers = 0;
  while(1) {
    if (triggers < thresholdTrigger.getNumberOfTriggers()){
      Serial.print("Number of Triggers: ");
      Serial.println(thresholdTrigger.getNumberOfTriggers());
      triggers = thresholdTrigger.getNumberOfTriggers();
    }
    thresholdTrigger.run();
    
    if (Serial_INTERRUPT_FLAG) {
      thresholdTrigger.end();
      Serial.println("Exiting Lightning Mode");
      return;
    }
  }
}

void soundMode() {
  setupSoundMode();
  Serial.println("Sound Mode");


  thresholdTrigger.resetCalibration();
  ADCSRA |= B01000000;
  while(!thresholdTrigger.isCalibrated())
    thresholdTrigger.calibrateThreshold();

  Serial.println("Waiting for Sound");
  thresholdTrigger.triggerCamera();
  delay(1000);
  
  Serial_INTERRUPT_FLAG = false;
  while(!thresholdTrigger.run()) {
    if (Serial_INTERRUPT_FLAG) {
      thresholdTrigger.end();
      Serial.println("Exiting Sound Mode");
      return;
              
    }
  }
  thresholdTrigger.end();
  Serial.println("Sound Triggered");  
}

void timelapseMode() {
  
  Serial.write("Timelapse Mode");
  Serial_INTERRUPT_FLAG = false;   //TODO: need to find out why Serial flag is being triggered from above line
  timelapse.reset();

  timelapse.setExposure(SerialRxMessage["exposure"]);
  timelapse.setTotalPhotos(SerialRxMessage["photos"]);
  timelapse.setDelayBetweenShots(SerialRxMessage["delay"]);
  
  
  while (!timelapse.isDone()) {
    timelapse.run();
    if (Serial_INTERRUPT_FLAG) {
      readSerialString();
      if (SerialRxMessage["flag"] == "err") {  // if the eit time-lapse command was received from phone
        timelapse.end();
        Serial_INTERRUPT_FLAG = false;
        break;
      }
      Serial_INTERRUPT_FLAG = false; // reset interrupt flag
    }
  }
  
  if (timelapse.isDone())
    Serial.write("Complete");
  else 
    Serial.write("Exited");
}

void hdrMode() {
  Serial.println("HDR Mode");
  Serial_INTERRUPT_FLAG = false;

  hdr.setPhotoTimeExposures();

  Serial.println("Running HDR");

  hdr.reset();

  delay(2000);
  while (!hdr.isDone()) {
    hdr.run();
  }
  Serial.println("HDR Done");
}

void singleShotMode() {

  if (SerialRxMessage["timed"] == false)
  {
    triggerCamera();
    _delay_ms(200);
    releaseCamera();
  }
  else {
    //statusLEDDoubleFlash();
    long startMillis = millis();
    long timerDelay = SerialRxMessage["delay"];
    triggerCamera();
    while((millis() - startMillis) < timerDelay) {
      if (Serial_INTERRUPT_FLAG) {
        readSerialString();
        if (SerialRxMessage["flag"] == "err") {  // if the eit time-lapse command was received from phone
          Serial_INTERRUPT_FLAG = false;
          break;
        }
        Serial_INTERRUPT_FLAG = false; // reset interrupt flag
      }
    }
    releaseCamera();
  }
  

}

void readSerialString() {
  String incommingMessage = ""; //clears variable for new input
  char c = ' ';
  while (c != '}') {
    // TODO: Add timeout timer for this
    if(Serial.available()) {
      c = Serial.read();  //gets one byte from serial buffer
      incommingMessage += c; 

      if (c == '}') {
        break;
      }  //breaks out of capture loop to print readstring
    } //makes the string readString  
  }

  if (incommingMessage.length() >0) {
    error = deserializeJson(SerialRxMessage, incommingMessage);

    if (error) {
      //Serial.write(F("deserializeJson() failed: "));
      //Serial.println(error.c_str());
      return;
    }
  }

  digitalWrite(statusLEDPin, LOW);
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

  sendAsyncInfo();
}

void sendAsyncInfo() {
  int admuxHolder = ADMUX;
  int adcsraHolder = ADCSRA;
  int adcsrbHolder = ADCSRB;
  String output;
  SerialTxMessage["batteryLevel"] = batteryIndicator.getBatteryLevelPercentage();
  SerialTxMessage["batteryStatus"] = batteryIndicator.getBatteryStatus();
  SerialTxMessage["currentMode"] = currentMode;
  //
  serializeJson(SerialTxMessage, output);
  Serial.println(output);

  ADMUX = admuxHolder;
  ADCSRA = adcsraHolder;
  ADCSRB = adcsrbHolder;
  ADCSRA |= B01000000;  // kick off next ADC 
}

void statusLEDDoubleFlash() {
  digitalWrite(statusLEDPin, HIGH);
  _delay_ms(300);
  digitalWrite(statusLEDPin, LOW);
  _delay_ms(300);
  digitalWrite(statusLEDPin, HIGH);
  _delay_ms(300);
  digitalWrite(statusLEDPin, LOW);

}
