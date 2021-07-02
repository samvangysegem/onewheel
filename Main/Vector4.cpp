
#include "Vector4.h"

RowVector4::RowVector4(){
    for (int i=0; i<4; i++){
        Vector[i] = 0;
    }
}

RowVector4::RowVector4(float arr[4]){
    for (int i=0; i<4; i++){
        Vector[i] = arr[i];
    }
}

const RowVector4 RowVector4::operator +(const RowVector4& secondOperand) const {
    // Arr for initialisation of new vector
    float resArr[4];
    // Calculate new values
    for (int i=0; i<4; i++){
        resArr[i] = Vector[i] + secondOperand.Vector[i];
    }
    // Return new vector
    return RowVector4(resArr);
}

const RowVector4 RowVector4::operator -(const RowVector4& secondOperand) const {
    // Arr for initialisation of new vector
    float resArr[4];
    // Calculate new values
    for (int i=0; i<4; i++){
        resArr[i] = Vector[i] - secondOperand.Vector[i];
    }
    // Return new vector
    return RowVector4(resArr);
}

ColumnVector4::ColumnVector4(){
    for (int i=0; i<4; i++){
        Vector[i] = 0;
    }
}

ColumnVector4::ColumnVector4(float arr[4]){
    for (int i=0; i<4; i++){
        Vector[i] = arr[i];
    }
}

const ColumnVector4 ColumnVector4::operator +(const ColumnVector4& secondOperand) const {
    // Arr for initialisation of new vector
    float resArr[4];
    // Calculate new values
    for (int i=0; i<4; i++){
        resArr[i] = Vector[i] + secondOperand.Vector[i];
    }
    // Return new vector
    return ColumnVector4(resArr);
}

const ColumnVector4 ColumnVector4::operator -(const ColumnVector4& secondOperand) const {
    // Arr for initialisation of new vector
    float resArr[4];
    // Calculate new values
    for (int i=0; i<4; i++){
        resArr[i] = Vector[i] - secondOperand.Vector[i];
    }
    // Return new vector
    return ColumnVector4(resArr);
}

// MULTIPLICATION OVERLOADING

const RowVector4 operator*(float scalar, const RowVector4& vec) {
    // Arr for initialisation of new vector
    float resArr[4];
    // Calculate new values
    for (int i=0; i<4; i++){
        resArr[i] = vec.getElement(i) * scalar;
    }
    // Return new vector
    return RowVector4(resArr);
}

const RowVector4 operator*(const RowVector4& vec, float scalar) {
    // Arr for initialisation of new vector
    float resArr[4];
    // Calculate new values
    for (int i=0; i<4; i++){
        resArr[i] = vec.getElement(i) * scalar;
    }
    // Return new vector
    return RowVector4(resArr);
}

const ColumnVector4 operator*(float scalar, const ColumnVector4& vec) {
    // Arr for initialisation of new vector
    float resArr[4];
    // Calculate new values
    for (int i=0; i<4; i++){
        resArr[i] = vec.getElement(i) * scalar;
    }
    // Return new vector
    return ColumnVector4(resArr);
}

const ColumnVector4 operator*(const ColumnVector4& vec, float scalar) {
    // Arr for initialisation of new vector
    float resArr[4];
    // Calculate new values
    for (int i=0; i<4; i++){
        resArr[i] = vec.getElement(i) * scalar;
    }
    // Return new vector
    return ColumnVector4(resArr);
}

const float operator*(const RowVector4& firstOperand, const ColumnVector4& secondOperand) {
    float res = 0;
    for (int i=0; i<4; i++){
        res += firstOperand.getElement(i) * secondOperand.getElement(i);
    }
    return res;
}

const Matrix4 operator*(const ColumnVector4& firstOperand, const RowVector4& secondOperand) {
    float resArr[4][4];
    for (int i=0; i<4; i++){
        for (int j=0; j<4; j++){
            resArr[i][j] = firstOperand.getElement(i) * secondOperand.getElement(j);
        }
    }
    return Matrix4(resArr);
}

const RowVector4 operator*(const RowVector4& firstOperand, const Matrix4& secondOperand) {
    float resArr[4];
    for (int i=0; i<4; i++){
        resArr[i] = 0;
        for (int j=0; j<4; j++){
            resArr[i] += firstOperand.getElement(j) * secondOperand.getElement(j, i);
        }
    }
    return RowVector4(resArr);
}

const ColumnVector4 operator*(const Matrix4& firstOperand, const ColumnVector4& secondOperand) {
    float resArr[4];
    for (int i=0; i<4; i++){
        resArr[i] = 0;
        for (int j=0; j<4; j++){
            resArr[i] += firstOperand.getElement(i, j) * secondOperand.getElement(j);
        }
    }
    return ColumnVector4(resArr);
}

