//
//  HDR.h
//
//  Created by Kane Stoboi on 21/04/2019.
//  Copyright © 2019 Kane Stoboi. All rights reserved.
//

#ifndef HDR
#define HDR_h

class HDR
{
  public:
    // constructor, requires two pointers to the shutter and release functions
    HDR(void (*cameraTriggerFunction)(), void (*cameraReleaseFunction)(), void (*cameraFocusFunction)());

    bool isDone();
    void reset();
    void run();


        // Function pointers
    void (*triggerCamera)();  // pointer to function that will trigger the camera to open the shutter
    void (*releaseCamera)();  // pointer to function that will release the camera shutter
    void (*focusCamera)();

  private:
    long photoTimeExposure[4] = { 500, 750, 1000};      //  the time exposure for each HDR photo
    long delayBetweenPhotos = 200;      // delay in milliseconds between each photo
    int focusDelay = 500;       // delay in milliseconds between focusing and triggering the camera shutter

    bool cameraTriggered = false;
    long timelapseTime;         // Time for pan to travel from zero to end (minutes)
    float exposureTime;         // Time that the shutter is open for (seconds)
    int totalPhotos;            // Total photos that need to be taken to complete the timelapse
    int photosTaken = 0;            // total photos that have already been taken durinig the timelapse
    long lastPhotoMillis = 0;   // the time since the start of program that the last photo was taken
    long triggeredMillis = 0;   // time since the start of program that the camera was triggered
    

};
#endif /* HDR_h */
