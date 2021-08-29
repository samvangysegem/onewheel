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

void Kalman::updateState(ColumnVector4& State, ColumnVector4& Obs_State, float Input) {
    // Predict
    H = Ad * P * Cd.T() * (V + Cd * P * Cd.T()).Inv();
    State = Ad * State + Bd * Input + H * (Obs_State - Cd * State);
    // Update
    P = (Ad - H * Cd) * P * Ad.T();
}

