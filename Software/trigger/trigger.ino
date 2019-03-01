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
int soundThreshold = 100;       // Threshold for sound to trigger camera
int lightningThreshold = 200;   // Threshold for light to trigger camera
int numberOfLightningTriggers = 0;

// High when a value is ready to be read
volatile int readFlag;

// Value to store analog result
volatile int analogVal;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  DDRB = DDRB | 0b00001111; // Ser focus and shutter pins to be outputs
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
  // put your main code here, to run repeatedly:
  // Check to see if the value has been updated

  //lightingMode();
  soundMode();
  
}

void ADCSetup() {
  // clear ADLAR in ADMUX (0x7C) to right-adjust the result
  // ADCL will contain lower 8 bits, ADCH upper 2 (in last two bits)
  ADMUX &= B11011111;
  
  // Set REFS1..0 in ADMUX (0x7C) to change reference voltage to the
  // proper source (01)
  ADMUX |= B01000000;
  
  // Clear MUX3..0 in ADMUX (0x7C) in preparation for setting the analog
  // input
  ADMUX &= B11110000;
  
  // Set MUX3..0 in ADMUX (0x7C) to read from AD8 (Internal temp)
  // Do not set above 15! You will overrun other parts of ADMUX. A full
  // list of possible inputs is available in Table 24-4 of the ATMega328
  // datasheet
  //ADMUX |= 0;
  // ADMUX |= B00001000; // Binary equivalent
  
  // Set ADEN in ADCSRA (0x7A) to enable the ADC.
  // Note, this instruction takes 12 ADC clocks to execute
  ADCSRA |= B10000000;
  
  // Set ADATE in ADCSRA (0x7A) to enable auto-triggering.
  ADCSRA |= B00000000;
  
  // Clear ADTS2..0 in ADCSRB (0x7B) to set trigger mode to free running.
  // This means that as soon as an ADC has finished, the next will be
  // immediately started.
  ADCSRB &= B11111000;
  
  // Set the Prescaler to 128 (16000KHz/128 = 125KHz)
  // Above 200KHz 10-bit results are not reliable.
  ADCSRA |= B00000111;
  
  // Set ADIE in ADCSRA (0x7A) to enable the ADC interrupt.
  // Without this, the internal interrupt will not trigger.
  ADCSRA |= B00001000;
}


// Interrupt service routine for the ADC completion
ISR(ADC_vect){
  analogVal = ADCL | (ADCH << 8); // Must read low byte first

  if (analogVal >= threshold) 
    trigger = true;
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

void soundMode() {
  ADMUX |= 0; // Set multiplexer to 0

  threshold = soundThreshold;
  
  // Enable global interrupts
  // AVR macro included in <avr/interrupts.h>, which the Arduino IDE
  // supplies by default.
  sei();
  
  // Kick off the first ADC
  // Set ADSC in ADCSRA (0x7A) to start the ADC conversion
  ADCSRA |=B01000000;


  while(true) {

    //Serial.print("");
    
    if (trigger == true) {

      triggerCamera();
      updateDisplay();

      _delay_ms(500);
      ADCSRA |= B01000000;  // kick off next ADC conversion

    }
    

    //Serial.println("");
  }
}

void lightingMode() {

  ADMUX |= 0; // Set multiplexer to 0
  
  // Enable global interrupts
  // AVR macro included in <avr/interrupts.h>, which the Arduino IDE
  // supplies by default.
  sei();
  
  // Kick off the first ADC
  readFlag = 0;
  // Set ADSC in ADCSRA (0x7A) to start the ADC conversion
  ADCSRA |=B01000000;

  while(1) {

    if (analogVal >= soundThreshold) {
      PORTB = 0b00000001;
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      PORTB = 0b00000000;
      numberOfLightningTriggers++;
    }
    
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(numberOfLightningTriggers);
    display.display();
    Serial.println(analogVal);
  }
}

void triggerCamera() {
  PORTB = 0b00000001;
  __asm__("nop\n\t"); 
  __asm__("nop\n\t"); 
  __asm__("nop\n\t"); 
  __asm__("nop\n\t"); 
  PORTB = 0b00000000;
  numberOfLightningTriggers++;

  trigger = false;

  return;
}


void updateDisplay() {
  display.clearDisplay();
  display.setCursor(0,10);
  display.println(numberOfLightningTriggers);
  display.display();

  return;
}
