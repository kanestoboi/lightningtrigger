// Include libraries for LCD
#include <util/delay.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <StateMachine.h>
#include <SoftwareSerial.h>// import the serial library
#include "Timelapse.h"
#include "ThresholdTrigger.h"
#include "BatteryIndicator.h"
#include "HDR.h"

void triggerCamera();
void focusCamera();
void releaseCamera();
void triggerFlash();

volatile bool BLUETOOTH_INTERRUPT_FLAG;

volatile char bluetoothRxMessage;

/*********************************
 * DEVELOPMENT SETUP
 ********************************/

int rx = 2;  // software serial RX pin
int tx = 3;  // software serial TX pin 
long counter = 0; // debugging counter


/*********************************
 * CONFIGURATION SETUP
 ********************************/
/*int rx = 10;  // software serial RX pin
int tx = 11;  // software serial TX pin 
*/
#define OLED_RESET 4

#define F_CPU 16000000UL  // Define the microcontroller clock speed (16 MHz)


// Create various objects
Adafruit_SSD1306 display(OLED_RESET);   // create LCD object
SoftwareSerial Bluetooth(rx, tx);       // create bluetooth object
Timelapse timelapse = Timelapse(&triggerCamera, &releaseCamera);
ThresholdTrigger thresholdTrigger = ThresholdTrigger(&focusCamera, &triggerCamera, &releaseCamera, &triggerFlash);
BatteryIndicator batteryIndicator = BatteryIndicator(7);
HDR hdr = HDR(&triggerCamera, &releaseCamera, &focusCamera);

// Variables
int lightningThreshold = 1000;  // Threshold for light to trigger camera
int soundThreshold = 740;
uint8_t batteryLevel = 0;

// Various Flags


// masks used
int cameraFocusMask = 0b00000001;
int cameraTriggerMask = 0b00000010; // PORT B Mask
int flashTriggerMask = 0b00001100;  // PORT B Mask
int switchesMask = 0b01111100;
int downMask = 0b00000100;
int centerMask = 0b00001000;
int leftMask = 0b00010000;
int upMask = 0b00100000;
int rightMask = 0b01000000;

void setup() {
  Serial.begin(9600);
  Bluetooth.begin(9600);  // Begin the bluetooth serial and set data rate
  Wire.begin();

  Bluetooth.println("Bluetooth connected");

  setSoundSensitivity(0);       // sets default sensitivity 
  setLightningSensitivity(127); // sets default sensitivity
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.clearDisplay();   // clear the adafruit splash screen
  
  /**********************************
   * THERE IS A BUG IN THE DISPLAY.BEGIN FUNCTION CAUSING PD4 TO BE SET HIGH
   * THE WORK AROUND BELOW SETS ALL PINS LOW BEFORE SETTING THEM AS INPUTS
   **********************************/
  DDRB = DDRB | 0b00001111; // set focus and shutter pins to be outputs
  //DDRD = 0b00000000;
  //PORTD = 0b00000000;
  //DDRD = DDRD | switchesMask; // set the 5-way switch pins to be inputs
  

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Lightning Trigger");
  display.setCursor(0,20);
  display.print("Threshold: ");
  display.println(lightningThreshold);
  display.display();


  attachInterrupt(0,bluetoothISR,  RISING); // setup interrupt for INT0 (UNO pin 2) //TODO: Change this to real interrupt
 


  //delay(4000);

  timer2InterruptSetup();

}



void loop() {
  //display.clearDisplay();
  //display.display();

  hdrMode();

  while(1);

  if (BLUETOOTH_INTERRUPT_FLAG) {
    if (bluetoothRxMessage == 't') {  // if the run time-lapse command was received from phone
      BLUETOOTH_INTERRUPT_FLAG = false;
      Bluetooth.println("tm");
      timelapseMode();
    }
    else if (bluetoothRxMessage == 's') {  // if the run sound mode command was received from phone
      BLUETOOTH_INTERRUPT_FLAG = false;
      Bluetooth.println("sm");
      soundMode();
    }
    else if (bluetoothRxMessage == 'l') {  // if the run lightning mode command was received from phone
      BLUETOOTH_INTERRUPT_FLAG = false;
      Bluetooth.println("lm");
      lightningMode();
    }
    else if (bluetoothRxMessage == 'h') {  // if the run HDR mode command was received from phone
      BLUETOOTH_INTERRUPT_FLAG = false;
      Bluetooth.println("hdr");
      hdrMode();
    }
    else {
      BLUETOOTH_INTERRUPT_FLAG = false;
    }   
  }

  //timelapseMode();
  
  //lightningMode();  
  
}

void bluetoothISR(){
  BLUETOOTH_INTERRUPT_FLAG = true; // trigger flag indicating a bluetoothRxMessage was received 
  bluetoothRxMessage = Bluetooth.read();  // read the messgage
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
  thresholdTrigger.setTriggerThreshold(soundThreshold);
  thresholdTrigger.setup();
  thresholdTrigger.reset();
  
}

void setupLightningMode() {
  thresholdTrigger.setADCInput(0); // Set multiplexer to 0
  thresholdTrigger.setTriggerThreshold(lightningThreshold);
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
}

void triggerFlash() {
  PORTB |= flashTriggerMask;  // trigger the outputs
  _delay_ms(200);
  PORTB = 0b00000000;   // reset trigger outputs to off
}


void updateDisplay() {
  display.clearDisplay();
  display.setCursor(0,20);
  display.println(thresholdTrigger.getNumberOfTriggers());
  display.display();
}

void lightningMode() {
  display.clearDisplay();
  display.display();
  display.setCursor(0,0);
  display.println("Lightning Trigger Mode");
  display.display();
  
  setupLightningMode();
  Bluetooth.println("Lightning Mode");
  BLUETOOTH_INTERRUPT_FLAG = false; //TODO: need to find out why bluetooth flag is being triggered from above line
  while (1) {
    if (BLUETOOTH_INTERRUPT_FLAG) {
      if (bluetoothRxMessage == 'r') {  // if the run time-lapse command was received from phone
        BLUETOOTH_INTERRUPT_FLAG = false;
        Bluetooth.println("Run Lightning Mode");
        break;
      }
      else {
        BLUETOOTH_INTERRUPT_FLAG = false;
        return;
      }   
    }
  }

  thresholdTrigger.resetCalibration();
  ADCSRA |= B01000000;
  while(!thresholdTrigger.isCalibrated())
    thresholdTrigger.calibrateThreshold();

  Bluetooth.println("Calibrated  Lightning Mode");

  display.clearDisplay();
  display.display();
  display.setCursor(0,0);
  display.println("Lightning Trigger Running");
  display.display();
  Serial.println("calibration done");
  ADCSRA |= B01000000;
  thresholdTrigger.focusCamera();
  while(1) {
    thresholdTrigger.run();
    updateDisplay();
    //Bluetooth.print(thresholdTrigger.getNumberOfTriggers());
    //Bluetooth.print(" | ");
    //Bluetooth.println(thresholdTrigger.analogVal());
    if (BLUETOOTH_INTERRUPT_FLAG) {
      if (bluetoothRxMessage == 'e') {  // if the eit time-lapse command was received from phone
        BLUETOOTH_INTERRUPT_FLAG = false;
        break;
      }
      BLUETOOTH_INTERRUPT_FLAG = false; // reset interrupt flag
    }
  }
}

void soundMode() {
  setupSoundMode();
  Serial.println("Sound Mode");

  
  
  display.clearDisplay();
  display.display();
  display.setCursor(0,0);
  display.println("Sound Trigger Running");
  display.display();

  while(1) {

  while (1) {
    if (BLUETOOTH_INTERRUPT_FLAG) {
      if (bluetoothRxMessage == 'r') {  // if the run time-lapse command was received from phone
        BLUETOOTH_INTERRUPT_FLAG = false;
        break;
      }
      else if (bluetoothRxMessage == 'e'){
        BLUETOOTH_INTERRUPT_FLAG = false;
        return;
      }   
    }
  }

  thresholdTrigger.resetCalibration();
  ADCSRA |= B01000000;
  while(!thresholdTrigger.isCalibrated())
    thresholdTrigger.calibrateThreshold();

  Bluetooth.println("Waiting for Sound");
  
  thresholdTrigger.triggerCamera();
  while(!thresholdTrigger.run()) {
    if (BLUETOOTH_INTERRUPT_FLAG) {
      if (bluetoothRxMessage == 'e') {  // if the run time-lapse command was received from phone
        BLUETOOTH_INTERRUPT_FLAG = false;
        break;
      }
      
      BLUETOOTH_INTERRUPT_FLAG = false;
      
    }
//    Bluetooth.print(THRESHOLD_TRIGGER_THRESHOLD);
//    Bluetooth.print(" | ");
//    Bluetooth.println(thresholdTrigger.analogVal());
  }
  thresholdTrigger.releaseCamera();

  Bluetooth.println("Sound Triggered");
  }
  
}

void timelapseMode() {
  
  Bluetooth.println("Timelapse Mode");
  BLUETOOTH_INTERRUPT_FLAG = false;   //TODO: need to find out why bluetooth flag is being triggered from above line
  timelapse.reset();
  while (1) {
    if (BLUETOOTH_INTERRUPT_FLAG) {
      if (bluetoothRxMessage == 'r') {  // if the run time-lapse command was received from phone
        BLUETOOTH_INTERRUPT_FLAG = false;
        break;
      }
      else if (bluetoothRxMessage == '1') {
        timelapse.setTimelapseTime(timelapse.getTimelapseTime() + 1);
        Bluetooth.println(timelapse.getTimelapseTime());
        BLUETOOTH_INTERRUPT_FLAG = false;
      }
      else {
        BLUETOOTH_INTERRUPT_FLAG = false;
        return;
      }     
    }  
  }
  
  while (!timelapse.isDone()) {
    timelapse.run();
    if (BLUETOOTH_INTERRUPT_FLAG) {
      if (bluetoothRxMessage == 'e') {  // if the eit time-lapse command was received from phone
        timelapse.end();
        BLUETOOTH_INTERRUPT_FLAG = false;
        break;
      }
      BLUETOOTH_INTERRUPT_FLAG = false; // reset interrupt flag
    }
  }
  
  if (timelapse.isDone())
    Bluetooth.println("Complete");
  else 
    Bluetooth.println("Exited");
  
}

void hdrMode() {
  Serial.println("HDR Mode");

  delay(2000);
  while (!hdr.isDone()) {
    hdr.run();
  }
  Serial.println("HDR Done");
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
  
  batteryLevel = batteryIndicator.getBatteryLevelPercentage();

  /*
  Bluetooth.print("Battery Level: ");
  Bluetooth.print(batteryIndicator.getBatteryLevelPercentage());
  Bluetooth.println("%");
  //setupLightningMode();*/
  
}
