// Include libraries for LCD
#include <util/delay.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <StateMachine.h>
#include <SoftwareSerial.h>// import the serial library
#include "Timelapse.h"
#include "ThresholdTrigger.h"

void triggerCamera();
void releaseCamera();


#define OLED_RESET 4

#define F_CPU 16000000UL  // Define the microcontroller clock speed (16 MHz)

long counter = 0; // debugging counter

// Create various objects
Adafruit_SSD1306 display(OLED_RESET);   // create LCD object
StateMachine machine = StateMachine();  // Create state machine object

Timelapse timelapse = Timelapse(&triggerCamera, &releaseCamera);
ThresholdTrigger thresholdTrigger = ThresholdTrigger(&triggerCamera, &releaseCamera);



long lastMillis = millis();

volatile bool trigger = false;  // Has the trigger threshold been met?
int threshold = 1000;           // The level that the ADC needs to reach before the camera is triggered
int sensitivity = 10;           // the amount above the ADC value to trigger the camera
int soundThreshold = 761;       // Threshold for sound to trigger camera
int lightningThreshold = 1000;  // Threshold for light to trigger camera
int numberOfTriggers = 0;       // Total times the camera has been triggered
volatile int output = 0;        // the continious filtered ADC reading used to update the threshold
volatile int adcCompleteFlag = 0;
int rx = 10;  // software serial RX pin
int tx = 11;  // software serial TX pin 
volatile bool BLUETOOTH_INTERRUPT_FLAG = false;
volatile char bluetoothRxMessage;

// masks used
int cameraTriggerMask = 0b00000011; // PORT B Mask
int flashTriggerMask = 0b00001100;  // PORT B Mask
int switchesMask = 0b01111100;
int downMask = 0b00000100;
int centerMask = 0b00001000;
int leftMask = 0b00010000;
int upMask = 0b00100000;
int rightMask = 0b01000000;

// Value to store analog result
volatile int analogVal;

// Define states for state machine
State* S0 = machine.addState(&lightningMode);
State* S1 = machine.addState(&soundMode);
/*State* S3 = machine.addState(&state3);
State* S4 = machine.addState(&state4);
State* S5 = machine.addState(&state5);
*/

SoftwareSerial Bluetooth(3, 4); // RX, TX

void setup() {
  Serial.begin(9600);
  Bluetooth.begin(9600);  // Begin the bluetooth serial and set data rate
  Wire.begin();

  Bluetooth.println("Bluetooth connected");

  thresholdTrigger.setup(); // Setup registers for ADC

  setSoundSensitivity(0);       // sets default sensitivity 
  setLightningSensitivity(128); // sets default sensitivity

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

  setupLightningMode();   // lightning trigger is the default mode on start up

  createTransitions();    // setup state transitions

  attachInterrupt(0,bluetoothISR,  RISING); // setup interrupt for INT0 (UNO pin 2)
 
  


  //delay(4000);
  
}

void loop() {

  //machine.run();
  
  //getKeyPress();
  
  //setupLightningMode();
  //setupSoundMode();
  //calibrateThreshold();
  //timelapseMode();
  
  lightningMode();
  //runTrigger();
  Serial.println("exited runTrigger()");
  
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
}

void setupLightningMode() {
  thresholdTrigger.setADCInput(0); // Set multiplexer to 0
  thresholdTrigger.setTriggerThreshold(lightningThreshold);
}

void calibrateThreshold() {
  /*
  display.clearDisplay();
  display.display();
  display.setCursor(0,0);
  display.println("Calibrating");
*/
  
/*
  display.setCursor(60,20);
  display.println("Calibrated");
  display.display();
  */

  return;
  
}

void runTrigger() {
  sei();  // Enable global interrupts

  

  
  if ((ADMUX & 0b00000001) == 1) {
    triggerCamera();
  }
  
  delay(1000);
  ADCSRA |=B01000000; // Set ADSC in ADCSRA (0x7A) to start the ADC conversion

  while(true) {

    // if bluetooth interrupt has occured
    if (BLUETOOTH_INTERRUPT_FLAG && bluetoothRxMessage == 'e') {
      BLUETOOTH_INTERRUPT_FLAG = false;  // clear the interrupt trigger flag
      break;
    }
  
    
  }

  while (trigger == false); // wait for last ADC to complete
      trigger = false;  // reset trigger flag
}

static void triggerCamera() {
  PORTB |= cameraTriggerMask;  // trigger the outputs  
}

static void releaseCamera() {
  PORTB = 0b00000000;   // reset trigger outputs to off
}

void triggerFlash() {
  PORTB |= flashTriggerMask;  // trigger the outputs
  _delay_ms(100);
  PORTB = 0b00000000;   // reset trigger outputs to off
}


void updateDisplay() {
  display.fillRect(0, 20, 32, 20, BLACK);
  display.setCursor(0,20);
  display.println(numberOfTriggers);
  display.display();
}

char getKeyPress() {

  if (PIND != 0b00000000)
  _delay_ms(50);
  int dir = PIND & switchesMask;


  //while((PIND & switchesMask) != 0);

  _delay_ms(10);
  
  if (dir == downMask) {
    Serial.println("Down");
    return 'd';
  }
  else if (dir == centerMask) {
    Serial.println("Center");
    return 'c';
  }
  else if (dir == leftMask) {
    Serial.println("Left");
    return 'l';
  }
  else if (dir == upMask) {
    Serial.println("Up");
    return 'u';
  }
  else if (dir == rightMask) {
    Serial.println("Right");
    return 'r';
  }
  return 0;
}

void lightningMode() {
  pinMode(A5, OUTPUT);
  digitalWrite(A5, HIGH);
  Serial.println("Lightning Mode");
  BLUETOOTH_INTERRUPT_FLAG = false; //TODO: need to find out why bluetooth flag is being triggered from above line
  while (1) {
    if (BLUETOOTH_INTERRUPT_FLAG) {
      if (bluetoothRxMessage == 'r') {  // if the run time-lapse command was received from phone
        BLUETOOTH_INTERRUPT_FLAG = false;
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
    
  Serial.println("calibration done");
  ADCSRA |= B01000000;
  while(1) {
    thresholdTrigger.run();

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
  Serial.println("Sound Mode");
  
  display.clearDisplay();
  display.display();
  display.setCursor(0,0);
  display.println("Sound Trigger Running");
  display.display();

  while (1) {
    if (BLUETOOTH_INTERRUPT_FLAG) {
      if (bluetoothRxMessage == 'r') {  // if the run time-lapse command was received from phone
        BLUETOOTH_INTERRUPT_FLAG = false;
        break;
      }
      else {
        BLUETOOTH_INTERRUPT_FLAG = false;
        return;
      }   
    }
  }
 
  while(1) {
    thresholdTrigger.run();
    //Serial.println(ANALOGUE_VAL);
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

void createTransitions() {
  /*************************
  * Lightning Trigger State Transitions
  */
  S0->addTransition([](){
    
    if (getKeyPress() == 'd'){
      
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      display.println("Sound Trigger");
      display.setCursor(0,20);
      display.print("Threshold: ");
      display.println(soundThreshold);
      display.display();

      setupSoundMode();
      
      return true;
    }
    return false;
  },S1);

  S0->addTransition([](){
    if (getKeyPress() == 'r'){
      if (lightningThreshold < 1022)
        lightningThreshold++;
      display.fillRect(0, 20, 128, 30, BLACK);
      display.setCursor(0,20);
      display.print("Threshold: ");
      display.println(lightningThreshold);
      display.display();
      return true;
    }
    return false;
  },S0);

  S0->addTransition([](){
    if (getKeyPress() == 'l'){
      if (lightningThreshold > 0)
        lightningThreshold--;
        
      display.fillRect(0, 20, 128, 30, BLACK);
      display.setCursor(0,20);
      display.print("Threshold: ");
      display.println(lightningThreshold);
      display.display();
      return true;
    }
    return false;
  },S0);

  /**********************
  * Sound Trigger State Transitions
  */
  S1->addTransition([](){
    if (getKeyPress() == 'u'){
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      display.println("Lightning Trigger");
      display.setCursor(0,20);
      display.print("Threshold: ");
      display.println(lightningThreshold);
      display.display();

      setupLightningMode();
      
      return true;
    }
    return false;
  },S0);
  
  S1->addTransition([](){
    if (getKeyPress() == 'r'){
      if (soundThreshold < 1022)
        soundThreshold++;
      display.fillRect(0, 20, 128, 30, BLACK);
      display.setCursor(0,20);
      display.print("Threshold: ");
      display.println(soundThreshold);
      display.display();
      return true;
    }
    return false;
  },S1);

  S1->addTransition([](){
    if (getKeyPress() == 'l'){
      if (soundThreshold > 0)
        soundThreshold--;
        
      display.fillRect(0, 20, 128, 30, BLACK);
      display.setCursor(0,20);
      display.print("Threshold: ");
      display.println(soundThreshold);
      display.display();
      return true;
    }
    return false;
  },S1);
}
