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

    
    int getBatteryLevelRaw();
    int getBatteryLevelPercentage();
    bool getBatteryStatus();

  private:

    uint8_t ADCPin;
    uint8_t admuxHolder;
    uint8_t adcsraHolder;
    uint8_t adcsrbHolder;
    int adcBatteryLevelMax = 910;
    int adcBatteryLevelMin = 987;
    double percentageCalcConstant = 100.0 / double(adcBatteryLevelMax - adcBatteryLevelMin);

    void batteryPercentageSetup();
    void batteryStatusSetup();

};
#endif /* BatteryIndicator_h */
