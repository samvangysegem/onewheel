#ifndef _Kalman_h_
#define _Kalman_h_

#include "Matrix4.h"
#include "Vector4.h"

class Kalman {
  public:
      // Constructor for this class
      Kalman();
  
  public:
      // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
      float getAngle(float newAngle, float newRate, float dt);
  
      void setAngle(float angle) { this->angle = angle; }; // Used to set angle, this should be set as the starting angle
      float getRate() { return this->rate; }; // Return the unbiased rate
  
      /* These are used to tune the Kalman filter */
      void setQangle(float Q_angle) { this->Q_angle = Q_angle; };
      void setQbias(float Q_bias) { this->Q_bias = Q_bias; };
      void setRmeasure(float R_measure) { this->R_measure = R_measure; };
  
      float getQangle() { return this->Q_angle; };
      float getQbias() { return this->Q_bias; };
      float getRmeasure() { return this->R_measure; };
  
      /**
       * setQbias(float Q_bias)
       * Default value (0.003f) is in Kalman.cpp. 
       * Raise this to follow input more closely,
       * lower this to smooth result of kalman filter.
       */
  
  private:
        // Model matrices
        Matrix4 Ad;
        ColumnVector4 Bd;
        Matrix4 Cd;
        // Stricly speaking, Cd is not a square matrix since not all variables are measured
        // yet velocity state variables can be derived from the position state variables

        // Covariance matrix of the measurement noise
        Matrix4 V;
};

#endif
