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
    
    Timelapse();
    
    int getTimelapseTime();
    void setTimelapseTime(int time);
    
    float getExposure();
    void setExposure(float exposure);
    
    void setFrameRate(int direction);
    int getFrameRate();
    
    int getDelayBetweenShots();
    void setDelayBetweenShots(int time);
    
    long getEditedTimeLapseLength();

    void calculateTotalPhotos();
    long getTotalPhotos();
    
    void calculateEditedTimeLapseLength();

    void run();
    bool isDone();
    

  private:
    
    int timelapseTime; // Time for pan to travel from zero to end                     (minutes)
    int frameRate;      // the frame rate used for calculation of the edited time lapse (Frames Per Second)
    long editedTimeLapseLength; // length of the edited time lapse              (seconds)
    float exposureTime;  // Time that shutter is closed for                     (seconds)
    int totalPhotos;
    int photosTaken;
    long lastPhotoMillis = 0;
    int delayBetweenShots;
    bool timelapseComplete = false;

};
#endif /* TimeLapse_hpp */
