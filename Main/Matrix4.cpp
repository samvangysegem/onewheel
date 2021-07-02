
#include "Matrix4.h"

Matrix4::Matrix4(){
    for (int i=0; i<4; i++){
        for (int j=0; j<4; j++){
            Matrix[i][j] = 0;
        }
    }
}

Matrix4::Matrix4(float arr[4][4]){
    for (int i=0; i<4; i++){
        for (int j=0; j<4; j++){
            Matrix[i][j] = arr[i][j];
        }
    }
}

Matrix4 Matrix4::transpose(){
    // Create arr for initialisation of new matrix
    float resArr[4][4];
    // Store result in array
    for (int i=0; i<4; i++){
        for (int j=0; j<4; j++){
            resArr[j][i] = Matrix[i][j];
        }
    }
    // Return new matrix
    return Matrix4(resArr);
}

const Matrix4 Matrix4::operator +(const Matrix4& secondOperand) const {
    // Create arr for initialisation of new matrix
    float resArr[4][4];
    // Store result in array
    for (int i=0; i<4; i++){
        for (int j=0; j<4; j++){
            resArr[i][j] = Matrix[i][j] + secondOperand.Matrix[i][j];
        }
    }
    // Return new matrix
    return Matrix4(resArr);
}

const Matrix4 Matrix4::operator -(const Matrix4& secondOperand) const {
    // Create arr for initialisation of new matrix
    float resArr[4][4];
    // Store result in array
    for (int i=0; i<4; i++){
        for (int j=0; j<4; j++){
            resArr[i][j] = Matrix[i][j] - secondOperand.Matrix[i][j];
        }
    }
    // Return new matrix
    return Matrix4(resArr);
}

const Matrix4 Matrix4::operator *(const Matrix4& secondOperand) const {
    // Create arr for initialisation of new matrix
    float resArr[4][4];
    // Store result in array
    for (int i=0; i<4; i++){
        for (int j=0; j<4; j++){
            float res = 0;
            for (int k=0; k<4; k++){
                res += Matrix[i][k]*secondOperand.Matrix[k][j];
            }
            resArr[i][j] = res;
        }
    }
    // Return new matrix
    return Matrix4(resArr);
}

const Matrix4 operator*(float scalar, const Matrix4& matrix){
    // Create arr for initialisation of new matrix
    float resArr[4][4];
    // Store result in array
    for (int i=0; i<4; i++){
        for (int j=0; j<4; j++){
            resArr[i][j] = scalar * matrix.getElement(i, j);
        }
    }
    // Return new matrix
    return Matrix4(resArr);
}

const Matrix4 operator*(const Matrix4& matrix, float scalar){
    // Create arr for initialisation of new matrix
    float resArr[4][4];
    // Store result in array
    for (int i=0; i<4; i++){
        for (int j=0; j<4; j++){
            resArr[i][j] = scalar * matrix.getElement(i, j);
        }
    }
    // Return new matrix
    return Matrix4(resArr);
}