#ifndef _Kalman_h_
#define _Kalman_h_

#include "Matrix4.h"
#include "Vector4.h"

class Kalman {
  public:
        // Constructor for this class
        Kalman();
        Kalman(const float (&_Ad)[4][4], const float (&_Bd)[4], const float (&_Cd)[4], const float (&_V)[4]);
  
  public:
        // Update filter and return state estimate
        ColumnVector4 updateFilter(ColumnVector4& STATE, ColumnVector4& OBS_STATE, float INPUT);
  
  private:
        // Model matrices
        Matrix4 Ad;
        ColumnVector4 Bd;
        Matrix4 Cd;
        // Stricly speaking, Cd is not a square matrix since not all variables are measured
        // yet velocity state variables can be derived from the position state variables

        // Covariance matrix of the measurement noise
        Matrix4 V;

        // Kalman filter
        Matrix4 P;
        Matrix4 H;
};

#endif
