
#ifndef _LQR_h_
#define _LQR_h_

#include "Matrix4.h"
#include "Vector4.h"

class LQR {

    public:
        LQR();
        LQR(const float (&_Ad)[4][4], const float (&_Bd)[4], const float (&_Q)[4], const float &_R);
    
    public:
        // Set/Get methods
        float getControl(const float (&state)[4], const float (&ksi)[4]); // Don't change original arrays here

    private:
        // Model and control variables
        Matrix4 Ad;
        ColumnVector4 Bd;

        // Weight variables
        Matrix4 Q;
        float R;

};

#endif