
#include "Kalman.h"

Kalman::Kalman() {
    // Matrices automatically initialised with values zero
}

Kalman::Kalman(const float (&_Ad)[4][4], const float (&_Bd)[4], const float (&_Cd)[4], const float (&_V)[4]) {
    // Model matrices
    Ad = Matrix4(_Ad);
    Bd = ColumnVector4(_Bd);
    Cd = Matrix4(_Cd);
    V = Matrix4(_V);
    // Init covariance of state
    float P_COV[4] = {0.1, 0.1, 0.1, 0.1};
    P = Matrix4(P_COV);
}

ColumnVector4 Kalman::updateFilter(ColumnVector4& STATE, ColumnVector4& OBS_STATE, float INPUT) {
    // Predict and update scheme
    STATE = Ad * STATE + Bd * INPUT + H * (OBS_STATE - Cd * STATE);
}

