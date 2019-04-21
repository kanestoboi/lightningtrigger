//
//  BatteryIndicator.h
//
//  Created by Kane Stoboi on 21/04/2019.
//  Copyright © 2019 Kane Stoboi. All rights reserved.
//

#ifndef BatteryIndicator_h
#define BatteryIndicator_h

class BatteryIndicator
{
  public:
    // constructor, requires two pointers to the shutter and release functions
    BatteryIndicator(int pin);

    void setup();
    int getBatteryLevelRaw();
    int getBatteryLevelPercentage();

  private:
    uint8_t ADCPin;
    uint8_t admuxHolder;
    uint8_t adcsraHolder;
    uint8_t adcsrbHolder;
    int adcBatteryLevelMax = 614;
    int adcBatteryLevelMin = 1023;
    double percentageCalcConstant = 100.0 / double(adcBatteryLevelMax - adcBatteryLevelMin);

};
#endif /* BatteryIndicator_h */
