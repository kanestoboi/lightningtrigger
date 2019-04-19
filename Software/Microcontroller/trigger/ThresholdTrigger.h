//
//  timeLapse.hpp
//  CameraSlider
//
//  Created by Kane Stoboi on 23/06/2016.
//  Copyright © 2016 Kane Stoboi. All rights reserved.
//

#ifndef ThresholdTrigger_h
#define ThresholdTrigger_h





class ThresholdTrigger
{
  public:
    // constructor, requires two pointers to the shutter and release functions
    ThresholdTrigger(void (*triggerFunction)(), void (*releaseFunction)());

    void setup();
    void setADCInput(int input);
    void setTriggerThreshold(int threshold);
    void calibrateThreshold();
    int analogVal();
    int getNumberOfTriggers();
    int getThreshold();
    void resetCalibration();
    bool isCalibrated();

    // Function prototypes
    // setters and getters for variables


    // Function pointers
    void (*triggerCamera)();  // pointer to function that will trigger the camera to open the shutter
    void (*releaseCamera)();  // pointer to function that will release the camera shutter
    
    
    // functions to for time-lapse operation
    void run();      
    void end();

    
  private:
    int numberOfTriggers = 0;
    int triggerThreshold;
    int sensitivity = 10;
    bool thresholdCalibrated = false;

};
#endif /* ThresholdTrigger_h */
