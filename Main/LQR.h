
#ifndef _LQR_h_
#define _LQR_h_

#include "Matrix4.h"
#include "Vector4.h"

class LQR {

    public:
        LQR();
        LQR(const float _Ad[4][4], const float _Bd[4], const float _Q[4], const float _R);
    
    public:
        // Set/Get methods
        float getControl(float state[4], float ksi[4]);

    private:
        // Model and control variables
        Matrix4 Ad;
        ColumnVector4 Bd;

        // Weight variables
        Matrix4 Q;
        float R;

};

#endif