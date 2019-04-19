//
//  timeLapse.cpp
//
//  Created by Kane Stoboi on 16/04/2019.
//  Copyright © 2016 Kane Stoboi. All rights reserved.
//
#include <Arduino.h>
#include "Timelapse.h"

Timelapse::Timelapse(void (*triggerFunction)(), void (*releaseFunction)()) {
    exposureTime = 1.00;    // Time that shutter is open for                     (seconds)
    delayBetweenShots = 1;  // The time in seconds between photos
    timelapseTime = 20;     // total time in seconds to be taking photos
    photosTaken = 0;

    calculateTotalPhotos();
    
    this->triggerCamera = triggerFunction;  
    this->releaseCamera = releaseFunction;
}

/**
 Sets the real time pan length of the slide and calculate the edited length
 
 @param time: time in seconds before all photos are taken
 @return
 */
void Timelapse::setTimelapseTime(long time) {
    timelapseTime = time;            
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

void Timelapse::calculateTotalPhotos() {
    totalPhotos = (long)((((float)(timelapseTime))/(exposureTime+((float)(delayBetweenShots)))));
}

long Timelapse::getTotalPhotos() {
    totalPhotos = (long)((((float)(timelapseTime))*60.0/(exposureTime+((float)(delayBetweenShots)))));
    return totalPhotos;
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
