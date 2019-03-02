byte READPIN = A0;

// Include libraries for LCD
#include <util/delay.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4

#define F_CPU 16000000UL  // 1 MHz


// create LCD object
Adafruit_SSD1306 display(OLED_RESET);


long lastMillis = millis();

volatile bool trigger = false;
int threshold = 0;
int sensitivity = 0;
int soundThreshold = 105;       // Threshold for sound to trigger camera
int lightningThreshold = 20;   // Threshold for light to trigger camera
int numberOfTriggers = 0;

int triggerMask = 0b00000001;


// Value to store analog result
volatile int analogVal;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  DDRB = DDRB | 0b00001111; // set focus and shutter pins to be outputs
  DDRD = DDRD | 0b00000000; // set the 5-way switch pins to be inputs

  ADCSetup(); // Setup registers for ADC

  setSoundSensitivity(50);     // sets default sensitivity 
  setLightningSensitivity(50); // sets default sensitivity

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  display.clearDisplay();   // clear the adafruit splash screen

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Lightning Trigger");
  display.display();

  //delay(4000);
  
}

void loop() {
  

  

  //lightingMode();
  //setupLightningMode();
  //runTrigger();

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
  ADCSRA |= B00000111;

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
  Serial.println(analogVal);
  if (analogVal >= threshold){
     
    trigger = true;
  }
  else 
    ADCSRA |= B01000000;  // Set ADSC in ADCSRA (0x7A) to start another ADC conversion
}

void setSoundSensitivity(int val) {
  Wire.beginTransmission(0b01011111); // transmit to device 
  // device address is specified in datasheet
  Wire.write(byte(0x00));            // sends instruction byte
  Wire.write(val);             // sends potentiometer value byte
  Wire.endTransmission();     // stop transmitting
}

void setLightningSensitivity(int val) {
  Wire.beginTransmission(0b01011110); // transmit to device 
  // device address is specified in datasheet
  Wire.write(byte(0x00));            // sends instruction byte
  Wire.write(val);             // sends potentiometer value byte
  Wire.endTransmission();     // stop transmitting
}

void setupSoundMode() {
  ADMUX |= 0; // Set multiplexer to 1
  threshold = soundThreshold;
}

void setupLightningMode() {
  ADMUX |= 0; // Set multiplexer to 0
  threshold = lightningThreshold;
}

void runTrigger() {
  sei();  // Enable global interrupts

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Waiting for trigger");
  display.display();
  
  ADCSRA |=B01000000; // Set ADSC in ADCSRA (0x7A) to start the ADC conversion

  while(true) {
    
    if (trigger == true) {

      triggerCamera();
      updateDisplay();

      _delay_ms(500);
      ADCSRA |= B01000000;  // kick off next ADC conversion
    }  
  }
}

void triggerCamera() {
  PORTB = triggerMask;  // trigger the outputs
  __asm__("nop\n\t"); 
  __asm__("nop\n\t"); 
  __asm__("nop\n\t"); 
  __asm__("nop\n\t"); 
  PORTB = 0b00000000;   // reset trigger outputs to off
  numberOfTriggers++;

  trigger = false;      // clear the trigger flag ready for another ADC conversion

}


void updateDisplay() {
  display.fillRect(0, 10, 32, 10, BLACK);
  display.setCursor(0,10);
  display.println(numberOfTriggers);
  display.display();
}
