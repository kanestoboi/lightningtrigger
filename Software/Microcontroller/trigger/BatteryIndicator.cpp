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

void BatteryIndicator::batteryPercentageSetup() {
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

  ADMUX |= 0b00000111; // Set ADC multiplexer ADC7
}

int BatteryIndicator::getBatteryLevelRaw() {
  this->batteryPercentageSetup();
  while((ADCSRA & 0b01000000) != 0b00000000); // wait for previous ADC to complete
  
  ADCSRA |= B01000000;  // kick off ADC

  while((ADCSRA & 0b01000000) != 0b00000000); // wait for previous ADC to complete

  ADMUX = this->admuxHolder;
  ADCSRA = this->adcsraHolder;
  ADCSRB = this->adcsrbHolder;

  ADCSRA |= B01000000;  // kick off next ADC

  return (ADCL | (ADCH << 8)); // return the complete ADC

  
}

int BatteryIndicator::getBatteryLevelPercentage() {
  return int(this->percentageCalcConstant * double(this->getBatteryLevelRaw() - this->adcBatteryLevelMin)); //TODO: Analyse the computation of this equation  
}


void BatteryIndicator::batteryStatusSetup() {
  this->admuxHolder = ADMUX;
  this->adcsraHolder = ADCSRA;
  this->adcsrbHolder = ADCSRB;

  // clear ADLAR in ADMUX (0x7C) to right-adjust the result
  // ADCL will contain lower 8 bits, ADCH upper 2 (in last two bits)
  ADMUX &= 0b11010000;
  
  // Set REFS1 and REFS0 in ADMUX (bit 7 and 6) to change reference voltage to the
  // AVCC with external capacitor at the AREF pin is used as VRef
  ADMUX |= 0b01000000;
  
  // Clear MUX3..0 in ADMUX (0x7C) in preparation for setting the analog
  // input
  ADMUX &= 0b11110000;

  // Set the Prescaler to 128 (16000KHz/128 = 125KHz)
  // Above 200KHz 10-bit results are not reliable.
  ADCSRA = 0b00000111;

  // Set ADIE in ADCSRA (0x7A) to disable the ADC interrupt.
  // Without this, the internal interrupt will not trigger.
  ADCSRA &= 0b11110111;
  
  // Set ADEN in ADCSRA (0x7A) to enable the ADC.
  // Note, this instruction takes 12 ADC clocks to execute
  ADCSRA |= 0b10000000;
  
  // Set ACME high for multiplexer to select negative input of analogue comparator
  ADCSRB &= 0b01000000;

  ADMUX |= 0b00000110; // Set ADC multiplexer
}

bool BatteryIndicator::getBatteryStatus() {
  this->batteryStatusSetup();
  while((ADCSRA & 0b01000000) != 0b00000000); // wait for previous ADC to complete
  
  ADCSRA |= B01000000;  // kick off ADC

  while((ADCSRA & 0b01000000) != 0b00000000); // wait for previous ADC to complete

  ADMUX = this->admuxHolder;
  ADCSRA = this->adcsraHolder;
  ADCSRB = this->adcsrbHolder;

  ADCSRA |= B01000000;  // kick off next ADC
  ADCSRA |= 0b00001000;

  return ((ADCL | (ADCH << 8)) > 100); // return the complete ADC
}