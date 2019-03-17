// Include libraries for LCD
#include <util/delay.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <StateMachine.h>

#define OLED_RESET 4

#define F_CPU 16000000UL  // Define the microcontroller clock speed (16 MHz)



Adafruit_SSD1306 display(OLED_RESET);   // create LCD object
StateMachine machine = StateMachine();  // Create state machine object


long lastMillis = millis();

volatile bool trigger = false;  // Has the trigger threshold been met?
int threshold = 1000;           // The level that the ADC needs to reach before the camera is triggered
int sensitivity = 10;           // the amount above the ADC value to trigger the camera
int soundThreshold = 761;       // Threshold for sound to trigger camera
int lightningThreshold = 1000;  // Threshold for light to trigger camera
int numberOfTriggers = 0;       // Total times the camera has been triggered
volatile int output = 0;        // the continious filtered ADC reading used to update the threshold
volatile int triggerFlag = 0; 

// masks used
int triggerMask = 0b00000011;
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
void setup() {
  Serial.begin(9600);
  Wire.begin();

  ADCSetup(); // Setup registers for ADC

  setSoundSensitivity(0);       // sets default sensitivity 
  setLightningSensitivity(128); // sets default sensitivity

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.clearDisplay();   // clear the adafruit splash screen

  /**********************************
   * THERE IS A BUG IN THE DISPLAY.BEGIN FUNCTION CAUSING PD4 TO BE SET HIGH
   * THE WORK AROUND BELOW SETS ALL PINS LOW BEFORE SETTING THEM AS INPUTS
   **********************************/
  DDRB = DDRB | 0b00001111; // set focus and shutter pins to be outputs
  DDRD = 0b00000000;
  PORTD = 0b00000000;
  DDRD = DDRD | switchesMask; // set the 5-way switch pins to be inputs
  
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Lightning Trigger");
  display.setCursor(0,20);
  display.print("Threshold: ");
  display.println(lightningThreshold);
  display.display();

  setupLightningMode();   // lightning trigger is the default mode on start up

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


  //delay(4000);
  
}

void loop() {

  //machine.run();
  
  //getKeyPress();
  
  setupLightningMode();
  calibrateThreshold();
  //setupSoundMode();
  runTrigger();

}

void ADCSetup() {
  // clear ADLAR in ADMUX (0x7C) to right-adjust the result
  // ADCL will contain lower 8 bits, ADCH upper 2 (in last two bits)
  ADMUX &= B11011111;
  
  // Set REFS1 and REFS0 in ADMUX (bit 7 and 6) to change reference voltage to the
  // AVCC with external capacitor at the AREF pin is used as VRef
  ADMUX |= B01000000;
  
  // Clear MUX3..0 in ADMUX (0x7C) in preparation for setting the analog
  // input
  ADMUX &= B11110000;

  // Set the Prescaler to 128 (16000KHz/128 = 125KHz)
  // Above 200KHz 10-bit results are not reliable.
  ADCSRA = B00000111;

  // Set ADIE in ADCSRA (0x7A) to enable the ADC interrupt.
  // Without this, the internal interrupt will not trigger.
  ADCSRA |= B00001000;
  
  // Set ADEN in ADCSRA (0x7A) to enable the ADC.
  // Note, this instruction takes 12 ADC clocks to execute
  ADCSRA |= B10000000;
  
  // Set ACME high for multiplexer to select negative input of analogue comparator
  ADCSRB &= B01000000;
}

// Interrupt service routine for the ADC completion
ISR(ADC_vect){
  analogVal = ADCL | (ADCH << 8); // Must read low byte first
  //Serial.println(analogVal);
  triggerFlag = 1;
  if (analogVal >= threshold){
    trigger = true;
  }
  else 
    ADCSRA |= B01000000;  // Set ADSC in ADCSRA (0x7A) to start another ADC conversion

  //update the threshold
  output = int(0.505*(float)output + 0.495*(float)analogVal);
  threshold = output + sensitivity;
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
  ADMUX |= 1; // Set multiplexer to 1
  threshold = soundThreshold;
}

void setupLightningMode() {
  ADMUX |= 0; // Set multiplexer to 0
  threshold = lightningThreshold;
}

void calibrateThreshold() {
  ADCSRA |= B01000000;
  display.clearDisplay();
  display.display();
  display.setCursor(0,0);
  display.println("Calibrating");
  display.display();
  
  //int threshold = 1000;
  _delay_ms(3000);

  while (1) {
    if (triggerFlag == 1) {
      if (output == threshold){
        threshold = threshold + 10;
        break;
      }
      threshold = output;
      triggerFlag = 0;
      ADCSRA |= B01000000;
    }

    display.fillRect(0, 10, 32, 10, BLACK);
    display.setCursor(0,10);
    display.println(output);
    display.println(threshold);
    display.display();
  }

  display.setCursor(60,20);
  display.println("Calibrated");
  display.display();

  return;
  
}

void runTrigger() {
  sei();  // Enable global interrupts

  display.clearDisplay();
  display.display();
  _delay_ms(1000);
  display.setCursor(0,0);
  display.println("Waiting for trigger");
  display.setCursor(60,20);
  display.println(threshold);
  display.display();

  ADCSRA |=B01000000; // Set ADSC in ADCSRA (0x7A) to start the ADC conversion

  while(true) {
  display.fillRect(0, 10, 32, 10, BLACK);
  display.setCursor(0,10);
  display.println(analogVal);
  display.display();
    
    if (trigger == true) {
      
      triggerCamera();
      updateDisplay();

      _delay_ms(500);
      trigger = false;
      ADCSRA |= B01000000;  // kick off next ADC conversion
      
    }  
  }
}

void triggerCamera() {
  PORTB = triggerMask;  // trigger the outputs
  _delay_ms(60);
  PORTB = 0b00000000;   // reset trigger outputs to off
  numberOfTriggers++;

  trigger = false;      // clear the trigger flag ready for another ADC conversion
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
  Serial.println("Lightning Mode");
  if (getKeyPress() == 'c')
    runTrigger();
}

void soundMode() {
  Serial.println(soundThreshold);
  if (getKeyPress() == 'c')
    runTrigger();
}
