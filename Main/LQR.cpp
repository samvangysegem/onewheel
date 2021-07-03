#include "LQR.h"

LQR::LQR(){
    // Matrices are automatically initialised with values of zero
}

LQR::LQR(const float _Ad[4][4], const float _Bd[4], const float _Q[4], const float _R) {
    // Model matrices
    for (int i=0; i<4; i++){
        Bd.setElement(i, _Bd[i]);
        Q.setElement(i, i, _Q[i]);
        for (int j=0; j<4; j++){
            Ad.setElement(i, j, _Ad[i][j]);
        }
    }
    R = _R;
}

float LQR::getControl(float state[4], float ksi[4]) {
    // Create vectors
    ColumnVector4 X(state);
    ColumnVector4 KSI(ksi);

    // Ricatti recursion
    Matrix4 M = Q;
    Matrix4 Mprev;
    for (int i=0; i<100; i++){
        Mprev = M;
        M = Q + (Ad.T()*Mprev*Ad) - (Ad.T()*Mprev*Bd*(1/(Bd.T()*Mprev*Bd+R))*Bd.T()*Mprev*Ad);
    }

    // Disturbance vector
    float eye[4] {1, 1, 1, 1};
    Matrix4 I(eye);
    ColumnVector4 G = (Ad - I)*KSI; // Constant disturbance
    Matrix4 temp = Ad-(Bd*(1/(R+Bd.T()*M*Bd)))*Bd.T()*M*Ad; // Common expression
    ColumnVector4 r = (I-temp.T()).Inv() * temp.T()*M*G; // Actual disturbance vector
    
    // Motor input voltage
    float u = ((-1)/(R+Bd.T()*M*Bd))*Bd.T()*(M*Ad*(X-KSI)+M*G+r);
    return u;
}