//
//  timeLapse.hpp
//  CameraSlider
//
//  Created by Kane Stoboi on 23/06/2016.
//  Copyright © 2016 Kane Stoboi. All rights reserved.
//

#ifndef TimeLapse_hpp
#define TimeLapse_hpp


class Timelapse
{
    
  public:
    // constructor, requires two pointers to the shutter and release functions
    Timelapse(void (*triggerFunction)(), void (*releaseFunction)());

    // Function prototypes
    void setTimelapseTime(long time);
    int getTimelapseTime();
    void setExposure(float exposure);
    float getExposure();
    void setDelayBetweenShots(int time);
    int getDelayBetweenShots();
    void calculateTotalPhotos();
    long getTotalPhotos();
    void (*triggerCamera)();  // pointer to function that will trigger the camera to open the shutter
    void (*releaseCamera)();  // pointer to function that will release the camera shutter
    void run();      
    void reset();
    bool isDone();
    
  private:
    long timelapseTime;         // Time for pan to travel from zero to end (minutes)
    float exposureTime;         // Time that shutter is closed for              (seconds)
    int totalPhotos;            // Total photos that need to be taken to complete the timelapse
    int photosTaken;            // total photos that have already been taken durinig the timelapse
    long lastPhotoMillis = 0;   // the time since the start of program that the last photo was taken
    long triggeredMillis = 0;   // time since the start of program that the camera was triggered
    long delayBetweenShots;     // the delay between the release of shutter to trigger of shutter
    bool cameraTriggered = false; // is the camera currently triggered?
    

};
#endif /* TimeLapse_hpp */
