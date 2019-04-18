//
//  timeLapse.cpp
//
//  Created by Kane Stoboi on 16/04/2019.
//  Copyright © 2016 Kane Stoboi. All rights reserved.
//
#include <Arduino.h>
#include "Timelapse.h"

Timelapse::Timelapse() {
    frameRate = 24;         // the frame rate used for calculation of the edited time lapse (Frames Per Second)
    exposureTime = 1.00;    // Time that shutter is open for                     (seconds)
    delayBetweenShots = 1;  // The time in seconds between photos
    timelapseTime = 60;     // total time in seconds to be taking photos
}

Timelapse::Timelapse(void (*trigger)(), void (*release)()) {
    frameRate = 24;         // the frame rate used for calculation of the edited time lapse (Frames Per Second)
    exposureTime = 1.00;    // Time that shutter is open for                     (seconds)
    delayBetweenShots = 1;  // The time in seconds between photos
    timelapseTime = 20;     // total time in seconds to be taking photos
    photosTaken = 0;

    calculateTotalPhotos();
    
    this->triggerCamera = trigger;
    this->releaseCamera = release;

}


/**
 Sets the real time pan length of the slide and calculate the edited length
 
 @param time: time in seconds before all photos are taken
 @return
 */
void Timelapse::setTimelapseTime(int time) {
    timelapseTime = time;
        
    calculateEditedTimeLapseLength();
    
    //Serial.println(editedTimeLapseLength);
}

/**
 gets the exposure for each shot in the time lapse
 
 @param void
 @return the exposure time currently set in seconds
 */
float Timelapse::getExposure() {
    return exposureTime;//
}

/**
 sets the exposure for each shot in the time lapse in seconds
 
 @param exposure is the exposure to be set in seconds
 @return void
 */
void Timelapse::setExposure(float exposure) {
    exposureTime = exposure;
    
}

/**
 gets the length of the dited time lapse
 
 @param void
 @return the length of the edited time lapse
 */
long Timelapse::getEditedTimeLapseLength() {
    calculateEditedTimeLapseLength();
    return editedTimeLapseLength;
}

/**
 sets the frameRate
 
 @param FPS 1 to increase the frame rate to the next
 @return void
 */
void Timelapse::setFrameRate(int FPS) {
    if (FPS == 1) {
        if (frameRate == 24) {
            frameRate = 25;
        }
        else if (frameRate == 25)
        {
            frameRate = 30;
        }
        else if (frameRate == 30)
        {
            frameRate = 48;
        }
        else if (frameRate == 48)
        {
            frameRate = 50;
        }
        else if (frameRate == 50)
        {
            frameRate = 60;
        }
    }
    
    if (FPS == 0) {
        if (frameRate == 60) {
            frameRate = 50;
        }
        else if (frameRate == 50)
        {
            frameRate = 48;
        }
        else if (frameRate == 48)
        {
            frameRate = 30;
        }
        else if (frameRate == 30)
        {
            frameRate = 25;
        }
        else if (frameRate == 25)
        {
            frameRate = 24;
        }
    }
    
    calculateEditedTimeLapseLength();
    
}

int Timelapse::getFrameRate() {
    return frameRate;
}

void Timelapse::calculateTotalPhotos() {
    totalPhotos = (long)((((float)(timelapseTime))/(exposureTime+((float)(delayBetweenShots)))));
}

long Timelapse::getTotalPhotos() {
    totalPhotos = (long)((((float)(timelapseTime))*60.0/(exposureTime+((float)(delayBetweenShots)))));
    return totalPhotos;
}

void Timelapse::calculateEditedTimeLapseLength() {
    editedTimeLapseLength = (long)(((float)(timelapseTime))*60.0/(float)(frameRate)/(exposureTime+(float)(delayBetweenShots)));
}

int Timelapse::getDelayBetweenShots() {
    return delayBetweenShots;
}

void Timelapse::setDelayBetweenShots(int time) {
    delayBetweenShots = time;
}

bool Timelapse::isDone() {
    return (totalPhotos == photosTaken);
}

void Timelapse::reset() {
  photosTaken = 0;
}

void Timelapse::run() {
  static bool initialRun = true;
  Serial.println(millis() - lastPhotoMillis);
    if (((millis() - lastPhotoMillis) > (delayBetweenShots * 1000)) && cameraTriggered == false || initialRun) {
      this->triggerCamera();
      cameraTriggered = true;
      triggeredMillis = millis();
      initialRun = false;
    }
    else if (cameraTriggered == true) {
      if (millis() - triggeredMillis > long(exposureTime * 1000.0)) {
        this->releaseCamera();
        photosTaken++;
        cameraTriggered = false;
        lastPhotoMillis = millis();        
      }
    }

}
