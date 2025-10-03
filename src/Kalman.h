#ifndef Kalman_h
#define Kalman_h
#include <arduino.h>


class Kalman 
{
  public:
      Kalman(float process_noise, float measurement_noise, float estimated_error, float initial_value);
      float update(float measurement);

  private:
    float q;  // Process noise covariance
    float r;  // Measurement noise covariance
    float x;  // Estimated value
    float p;  // Error covariance
    float k;  // Kalman gain
};
#endif