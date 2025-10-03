#include <arduino.h>
#include "Kalman.h"

Kalman::Kalman(float process_noise, float measurement_noise, float estimated_error, float initial_value) 
{
 q = process_noise;
 r = measurement_noise;
 x = initial_value;
 p = estimated_error;
 k = 0;
}
  
float Kalman::update(float measurement) 
{
// Prediction update
p = p + q;

// Measurement update
k = p / (p + r);  // Compute Kalman gain
x = x + k * (measurement - x);  // Update estimate
p = (1 - k) * p;  // Update error covariance
  
return x;  // Return filtered value
}