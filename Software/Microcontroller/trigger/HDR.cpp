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

void HDR::setCenterSpeed(int speed) {
  this->centerSpeed = speed;
  //this->setPhotoTimeExposures();
  // for (int i = 0; i < 3; i++) {
  //   Serial.println(this->photoTimeExposure[i]);
  // }
}

void HDR::setExposureValue(int ev) {
  this->exposureValue = ev;
}

void HDR::setPhotoTimeExposures() {
  this->photoTimeExposure[0] = (long)(1000.0 * (centerSpeed - (exposureValue/2.0 * centerSpeed)));
  this->photoTimeExposure[1] = (long)(1000.0 * centerSpeed);
  this->photoTimeExposure[2] = (long)(1000.0 * (centerSpeed + (exposureValue*2.0 * centerSpeed)));
  for (int i = 0; i < 3; i++) {
    Serial.println(this->photoTimeExposure[i]);
  }
}

bool HDR::isDone() {
    return (this->photosTaken == 3);
}

void HDR::reset() {
  this->initialRun = true;
  this->photosTaken = 0;
}

void HDR::run() {
	if (initialRun == true) {
		this->lastPhotoMillis = millis();
		this->releaseCamera();
		this->initialRun = false;
	}

	if (((millis() - this->lastPhotoMillis) > (this->delayBetweenPhotos + this->focusDelay)) && (this->cameraTriggered == false || this->initialRun)) {
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
      Serial.write("Photos Taken: ");
      Serial.println(photosTaken);        
    }
  }
  else if ((millis() - this->lastPhotoMillis) > (this->delayBetweenPhotos)) {
  	this->focusCamera();
  }
}
