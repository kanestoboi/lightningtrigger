//
//  ThresholdTrigger.cpp
//
//  Created by Kane Stoboi on 19/04/2019.
//  Copyright © 2016 Kane Stoboi. All rights reserved.
//



#include <Arduino.h>
#include <util/delay.h>
#include "ThresholdTrigger.h"
volatile int ANALOGUE_VAL = 0;
volatile int THRESHOLD_TRIGGER_FLAG = false;
volatile int THRESHOLD_TRIGGER_ADC_FLAG = false;
volatile int THRESHOLD_TRIGGER_THRESHOLD = 50;
volatile int THRESHOLD_TRIGGER_OUTPUT = 0;

 

ThresholdTrigger::ThresholdTrigger(void (*cameraFocusFunction)(), void (*cameraTriggerFunction)(), void (*cameraReleaseFunction)(), void (*flashTriggerFunction)()) {

    this->focusCamera = cameraFocusFunction;
    this->triggerCamera = cameraTriggerFunction;  
    this->releaseCamera = cameraReleaseFunction;
    this->triggerFlash = flashTriggerFunction;
}

void ThresholdTrigger::setup() {
  // clear ADLAR in ADMUX (0x7C) to right-adjust the result
  // ADCL will contain lower 8 bits, ADCH upper 2 (in last two bits)
  ADMUX &= B11011111;
  
  // Set REFS1 and REFS0 in ADMUX (bit 7 and 6) to change reference voltage to the
  // AVCC with external capacitor at the AREF pin is used as VRef
  ADMUX |= B01000000;
  
  
  

  // Set the Prescaler to 128 (16000KHz/128 = 125KHz)
  // Above 200KHz 10-bit results are not reliable.
  // ADC will do a conversion in about 152us with a clock of 16MHz
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

void ThresholdTrigger::reset() {
  this->numberOfTriggers = 0;
}

void ThresholdTrigger::setSensitivity(int s) {
  this->sensitivity = s;
}

void ThresholdTrigger::setTriggerDelay(int d) {
  this->triggerDelay = d;
}

void ThresholdTrigger::resetCalibration() {
  this->thresholdCalibrated = false;
}

bool ThresholdTrigger::isCalibrated() {
  return this->thresholdCalibrated;
}


void ThresholdTrigger::calibrateThreshold() {

  if (THRESHOLD_TRIGGER_FLAG)
    ADCSRA |= B01000000;
  
  if (THRESHOLD_TRIGGER_ADC_FLAG == true) {
    if (THRESHOLD_TRIGGER_THRESHOLD == THRESHOLD_TRIGGER_OUTPUT) {
      THRESHOLD_TRIGGER_THRESHOLD = THRESHOLD_TRIGGER_OUTPUT + this->sensitivity;
      THRESHOLD_TRIGGER_ADC_FLAG = false;
      while(!THRESHOLD_TRIGGER_FLAG); // wait for last last ADC conversion to complete
      THRESHOLD_TRIGGER_FLAG = false;
        
      this->thresholdCalibrated = true;

      return;
    }

    THRESHOLD_TRIGGER_THRESHOLD = THRESHOLD_TRIGGER_OUTPUT;
    THRESHOLD_TRIGGER_ADC_FLAG = 0;
    
  }

}

int ThresholdTrigger::analogVal() {
  return ANALOGUE_VAL;
}

int ThresholdTrigger::getNumberOfTriggers() {
  return this->numberOfTriggers;
}

int ThresholdTrigger::getThreshold() {
  return THRESHOLD_TRIGGER_THRESHOLD;
}

void ThresholdTrigger::setADCInput(int input) {
  // Clear MUX3..0 in ADMUX (0x7C) in preparation for setting the analog
  // input
  ADMUX &= B11110000;
  if (input >= 0 && input <= 7)
    ADMUX |= input; // Set ADC multiplexer 
}

void ThresholdTrigger::setTriggerThreshold(int threshold) {
  if (threshold >= 0 && threshold <= 1023)
    triggerThreshold = threshold; // Set triggering threshold  
}


void ThresholdTrigger::end() {
  ADCSRA &= B01111111;
  this->releaseCamera();
  ADCSRA &= B11110111;
}

bool ThresholdTrigger::run() {
  bool cameraWasTriggered = false;

  if (THRESHOLD_TRIGGER_FLAG == true) {
    cameraWasTriggered = true;
    
      if ((ADMUX & 0b00000001) == 0) {  // if lightning trigger mode trigger camera
        Serial.println("Triggered");
        this->triggerCamera();
        _delay_ms(200);
        this->releaseCamera();
      }
      else if ((ADMUX & 0b00000001) == 1) {  // if sound mode trigger flash
        delay(this->triggerDelay);
        this->triggerFlash();
        this->releaseCamera();
      }
      
      //_delay_ms(500); // TODO: remove this delay 
      this->numberOfTriggers++;
      Serial.println(this->getNumberOfTriggers());
      
      THRESHOLD_TRIGGER_FLAG = false;
      /*
      this->resetCalibration();
      ADCSRA |= B01000000;
      while(!this->isCalibrated())
        this->calibrateThreshold();
      */
      ADCSRA |= B01000000;  // kick off next ADC conversion

    }

    // If the ADC is complete update the threshold
    if (THRESHOLD_TRIGGER_ADC_FLAG == true) {
      THRESHOLD_TRIGGER_THRESHOLD = THRESHOLD_TRIGGER_OUTPUT + this->sensitivity;
      //Serial.println(THRESHOLD_TRIGGER_THRESHOLD);
      THRESHOLD_TRIGGER_ADC_FLAG = false;

      ADCSRA |= B01000000;  // kick off next ADC conversion
    }

    return cameraWasTriggered;
}

// Interrupt service routine for the ADC completion
ISR(ADC_vect){
  THRESHOLD_TRIGGER_ADC_FLAG = true;
  ANALOGUE_VAL = ADCL | (ADCH << 8); // Must read low byte first
  THRESHOLD_TRIGGER_OUTPUT = int(0.505*(float)THRESHOLD_TRIGGER_OUTPUT + 0.495*(float)ANALOGUE_VAL);
  if (ANALOGUE_VAL >= THRESHOLD_TRIGGER_THRESHOLD){
    THRESHOLD_TRIGGER_FLAG = true;
  }
  else {
    ADCSRA |= B01000000;  // Set ADSC in ADCSRA (0x7A) to start another ADC conversion
  }
  /*
  Serial.print(THRESHOLD_TRIGGER_THRESHOLD);
  Serial.print(" | ");
  Serial.println(ANALOGUE_VAL);*/
}
