//
//  ThresholdTrigger.hpp
//
//  Created by Kane Stoboi on 19/04/2019.
//  Copyright © 2016 Kane Stoboi. All rights reserved.
//

#ifndef ThresholdTrigger_h
#define ThresholdTrigger_h

extern volatile int ANALOGUE_VAL;
extern volatile int THRESHOLD_TRIGGER_FLAG;
extern volatile int THRESHOLD_TRIGGER_ADC_FLAG;
extern volatile int THRESHOLD_TRIGGER_THRESHOLD;
extern volatile int THRESHOLD_TRIGGER_OUTPUT;


class ThresholdTrigger
{
  public:
    // constructor, requires two pointers to the shutter and release functions
    ThresholdTrigger(void (*cameraTriggerFunction)(), void (*cameraReleaseFunction)(), void (*flashTriggerFunction)());

    void setup();
    void setADCInput(int input);
    void setTriggerThreshold(int threshold);
    void calibrateThreshold();
    int analogVal();
    int getNumberOfTriggers();
    int getThreshold();
    void reset();
    void resetCalibration();
    bool isCalibrated();

    // Function prototypes
    // setters and getters for variables


    // Function pointers
    void (*triggerCamera)();  // pointer to function that will trigger the camera to open the shutter
    void (*releaseCamera)();  // pointer to function that will release the camera shutter
    void (*triggerFlash)();
    
    
    // functions to for time-lapse operation
    bool run();      
    void end();

    
  private:
    int numberOfTriggers = 0;
    int triggerThreshold;
    int sensitivity = 10;
    bool thresholdCalibrated = false;

};
#endif /* ThresholdTrigger_h */
