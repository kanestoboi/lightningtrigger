//
//  HDR.cpp
//
//  Created by Kane Stoboi on 26/04/2019.
//  Copyright © 2016 Kane Stoboi. All rights reserved.
//



#include <Arduino.h>
#include "HDR.h"


HDR::HDR(void (*cameraTriggerFunction)(), void (*cameraReleaseFunction)(), void (*cameraFocusFunction)()) {

	this->triggerCamera = cameraTriggerFunction;
  this->releaseCamera = cameraReleaseFunction;
  this->focusCamera = cameraFocusFunction;
}

bool HDR::isDone() {
    return (this->photosTaken == 3);
}

void HDR::setCenter(int milliseconds) {
	this->photoTimeExposure[0] = milliseconds/2;
	this->photoTimeExposure[1] = milliseconds;
	this->photoTimeExposure[2] = milliseconds + milliseconds/2;
}

void HDR::reset() {
  this->photosTaken = 0;
}

void HDR::run() {
	static bool initialRun = true;
	if (initialRun == true) {
		this->lastPhotoMillis = millis();
		this->releaseCamera();
		initialRun = false;
	}

	if (((millis() - this->lastPhotoMillis) > (this->delayBetweenPhotos + this->focusDelay)) && (this->cameraTriggered == false || initialRun)) {
    this->triggerCamera();
    this->cameraTriggered = true;
    this->triggeredMillis = millis();
    initialRun = false;
	}
  else if (cameraTriggered == true) {
    if (millis() - triggeredMillis >= photoTimeExposure[photosTaken]) {
      this->releaseCamera();
      this->photosTaken++;
      this->cameraTriggered = false;
      this->lastPhotoMillis = millis();        
    }
  }
  else if ((millis() - this->lastPhotoMillis) > (this->delayBetweenPhotos)) {
  	this->focusCamera();
  }
}
