//
//  BatteryIndicator.cpp
//
//  Created by Kane Stoboi on 19/04/2019.
//  Copyright © 2016 Kane Stoboi. All rights reserved.
//



#include <Arduino.h>
#include "BatteryIndicator.h"


 

BatteryIndicator::BatteryIndicator(int pin) {
  this->ADCPin = pin;
}

void BatteryIndicator::setup() {
  this->admuxHolder = ADMUX;
  this->adcsraHolder = ADCSRA;
  this->adcsrbHolder = ADCSRB;
    
  // clear ADLAR in ADMUX (0x7C) to right-adjust the result
  // ADCL will contain lower 8 bits, ADCH upper 2 (in last two bits)
  ADMUX &= B11010000;
  
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
  ADCSRA &= B11110111;
  
  // Set ADEN in ADCSRA (0x7A) to enable the ADC.
  // Note, this instruction takes 12 ADC clocks to execute
  ADCSRA |= B10000000;
  
  // Set ACME high for multiplexer to select negative input of analogue comparator
  ADCSRB &= B01000000;

  ADMUX |= 0b00000011; // Set ADC multiplexer
}

int BatteryIndicator::getBatteryLevelRaw() {
  this->setup();
  while((ADCSRA & 0b01000000) != 0b00000000); // wait for previous ADC to complete
  
  ADCSRA |= B01000000;  // kick off ADC

  while((ADCSRA & 0b01000000) != 0b00000000); // wait for previous ADC to complete

  return (ADCL | (ADCH << 8)); // return the complete ADC

  ADMUX = this->admuxHolder;
  ADCSRA = this->adcsraHolder;
  ADCSRB = this->adcsrbHolder;
}

int BatteryIndicator::getBatteryLevelPercentage() {
  return int(this->percentageCalcConstant * float(this->getBatteryLevelRaw() - this->adcBatteryLevelMin)); //TODO: Analyse the computation of this equation
}
