//
//  Timelapse.cpp
//
//  Created by Kane Stoboi on 16/04/2019.
//  Copyright © 2016 Kane Stoboi. All rights reserved.
//
#include <Arduino.h>
#include "Timelapse.h"

Timelapse::Timelapse(void (*triggerFunction)(), void (*releaseFunction)()) {
    exposureTime = 1.00;    // Time that shutter is open for                     (seconds)
    delayBetweenShots = 1;  // The time in seconds between photos
    timelapseTime = 15;     // total time in seconds to be taking photos
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

long Timelapse::getTimelapseTime() {
  return timelapseTime;
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
    this->exposureTime = exposure;
    
}


void Timelapse::calculateTotalPhotos() {
    this->totalPhotos = (long)((((float)(this->timelapseTime))/(this->exposureTime + ((float)(this->delayBetweenShots)))));
}

void Timelapse::setTotalPhotos(int photos) {
  this->totalPhotos = photos;
}

long Timelapse::getTotalPhotos() {
    this->totalPhotos = (long)((((float)(timelapseTime))*60.0/(exposureTime+((float)(delayBetweenShots)))));
    return this->totalPhotos;
}

long Timelapse::getPhotosTaken() {
    return this->photosTaken;
}

int Timelapse::getDelayBetweenShots() {
    return this->delayBetweenShots;
}

void Timelapse::setDelayBetweenShots(int time) {
    this->delayBetweenShots = time;
}

bool Timelapse::isDone() {
    return (this->totalPhotos == this->photosTaken);
}

void Timelapse::end() {
  this->releaseCamera();
}

void Timelapse::reset() {
  this->photosTaken = 0;
}

void Timelapse::run() {
  static bool initialRun = true;
  if (((millis() - this->lastPhotoMillis) > (this->delayBetweenShots * 1000)) && this->cameraTriggered == false || initialRun) {
      this->triggerCamera();
      this->cameraTriggered = true;
      this->triggeredMillis = millis();
      initialRun = false;
    }
    else if (cameraTriggered == true) {
      if (millis() - triggeredMillis > long(exposureTime * 1000.0)) {
        this->releaseCamera();
        this->photosTaken++;
        this->cameraTriggered = false;
        this->lastPhotoMillis = millis();        
      }
    }

}
