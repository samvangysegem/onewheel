
#include "Matrix4.h"

Matrix4::Matrix4(){
    for (int i=0; i<4; i++){
        for (int j=0; j<4; j++){
            Matrix[i][j] = 0;
        }
    }
}

Matrix4::Matrix4(const float (&arr)[4]){
    for (int i=0; i<4; i++){
        Matrix[i][i] = arr[i];
        for (int j=0; j<4; j++){
            if (i != j) {
                Matrix[i][j] = 0;
            }
        }
    }
}

Matrix4::Matrix4(const float (&arr)[4][4]){
    for (int i=0; i<4; i++){
        for (int j=0; j<4; j++){
            Matrix[i][j] = arr[i][j];
        }
    }
}

Matrix4::Matrix4(const Matrix4& mat2){
    for (int i=0; i<4; i++){
        for (int j=0; j<4; j++){
            Matrix[i][j] = mat2.getElement(i, j);
        }
    }
}

Matrix4 Matrix4::T(){
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

Matrix4 Matrix4::Inv(){
    //
    // Inversion by Cramer's rule.  Code taken from: https://graphics.stanford.edu/~mdfisher/Code/Engine/Matrix4.cpp.html
    //
    float resArr[4][4];
    float tmp[12]; /* temp array for pairs */
    float src[16]; /* array of transpose source matrix */
    float det; /* determinant */
    /* transpose matrix */
    for (int i = 0; i < 4; i++)
    {
        src[i + 0 ] = Matrix[i][0];
        src[i + 4 ] = Matrix[i][1];
        src[i + 8 ] = Matrix[i][2];
        src[i + 12] = Matrix[i][3];
    }
    /* Calculate pairs for first 8 elements (cofactors) */
    tmp[0] = src[10] * src[15];
    tmp[1] = src[11] * src[14];
    tmp[2] = src[9] * src[15];
    tmp[3] = src[11] * src[13];
    tmp[4] = src[9] * src[14];
    tmp[5] = src[10] * src[13];
    tmp[6] = src[8] * src[15];
    tmp[7] = src[11] * src[12];
    tmp[8] = src[8] * src[14];
    tmp[9] = src[10] * src[12];
    tmp[10] = src[8] * src[13];
    tmp[11] = src[9] * src[12];

    /* Calculate first 8 elements (cofactors) */
    resArr[0][0] = tmp[0]*src[5] + tmp[3]*src[6] + tmp[4]*src[7];
    resArr[0][0] -= tmp[1]*src[5] + tmp[2]*src[6] + tmp[5]*src[7];
    resArr[0][1] = tmp[1]*src[4] + tmp[6]*src[6] + tmp[9]*src[7];
    resArr[0][1] -= tmp[0]*src[4] + tmp[7]*src[6] + tmp[8]*src[7];
    resArr[0][2] = tmp[2]*src[4] + tmp[7]*src[5] + tmp[10]*src[7];
    resArr[0][2] -= tmp[3]*src[4] + tmp[6]*src[5] + tmp[11]*src[7];
    resArr[0][3] = tmp[5]*src[4] + tmp[8]*src[5] + tmp[11]*src[6];
    resArr[0][3] -= tmp[4]*src[4] + tmp[9]*src[5] + tmp[10]*src[6];
    resArr[1][0] = tmp[1]*src[1] + tmp[2]*src[2] + tmp[5]*src[3];
    resArr[1][0] -= tmp[0]*src[1] + tmp[3]*src[2] + tmp[4]*src[3];
    resArr[1][1] = tmp[0]*src[0] + tmp[7]*src[2] + tmp[8]*src[3];
    resArr[1][1] -= tmp[1]*src[0] + tmp[6]*src[2] + tmp[9]*src[3];
    resArr[1][2] = tmp[3]*src[0] + tmp[6]*src[1] + tmp[11]*src[3];
    resArr[1][2] -= tmp[2]*src[0] + tmp[7]*src[1] + tmp[10]*src[3];
    resArr[1][3] = tmp[4]*src[0] + tmp[9]*src[1] + tmp[10]*src[2];
    resArr[1][3] -= tmp[5]*src[0] + tmp[8]*src[1] + tmp[11]*src[2];

    /* Calculate pairs for second 8 elements (cofactors) */
    tmp[0] = src[2]*src[7];
    tmp[1] = src[3]*src[6];
    tmp[2] = src[1]*src[7];
    tmp[3] = src[3]*src[5];
    tmp[4] = src[1]*src[6];
    tmp[5] = src[2]*src[5];

    tmp[6] = src[0]*src[7];
    tmp[7] = src[3]*src[4];
    tmp[8] = src[0]*src[6];
    tmp[9] = src[2]*src[4];
    tmp[10] = src[0]*src[5];
    tmp[11] = src[1]*src[4];

    /* Calculate second 8 elements (cofactors) */
    resArr[2][0] = tmp[0]*src[13] + tmp[3]*src[14] + tmp[4]*src[15];
    resArr[2][0] -= tmp[1]*src[13] + tmp[2]*src[14] + tmp[5]*src[15];
    resArr[2][1] = tmp[1]*src[12] + tmp[6]*src[14] + tmp[9]*src[15];
    resArr[2][1] -= tmp[0]*src[12] + tmp[7]*src[14] + tmp[8]*src[15];
    resArr[2][2] = tmp[2]*src[12] + tmp[7]*src[13] + tmp[10]*src[15];
    resArr[2][2] -= tmp[3]*src[12] + tmp[6]*src[13] + tmp[11]*src[15];
    resArr[2][3] = tmp[5]*src[12] + tmp[8]*src[13] + tmp[11]*src[14];
    resArr[2][3] -= tmp[4]*src[12] + tmp[9]*src[13] + tmp[10]*src[14];
    resArr[3][0] = tmp[2]*src[10] + tmp[5]*src[11] + tmp[1]*src[9];
    resArr[3][0] -= tmp[4]*src[11] + tmp[0]*src[9] + tmp[3]*src[10];
    resArr[3][1] = tmp[8]*src[11] + tmp[0]*src[8] + tmp[7]*src[10];
    resArr[3][1] -= tmp[6]*src[10] + tmp[9]*src[11] + tmp[1]*src[8];
    resArr[3][2] = tmp[6]*src[9] + tmp[11]*src[11] + tmp[3]*src[8];
    resArr[3][2] -= tmp[10]*src[11] + tmp[2]*src[8] + tmp[7]*src[9];
    resArr[3][3] = tmp[10]*src[10] + tmp[4]*src[8] + tmp[9]*src[9];
    resArr[3][3] -= tmp[8]*src[9] + tmp[11]*src[10] + tmp[5]*src[8];

    /* Calculate determinant */
    det = src[0]*resArr[0][0]+src[1]*resArr[0][1]+src[2]*resArr[0][2]+src[3]*resArr[0][3];

    /* Calculate matrix inverse */
    det = 1.0f / det;

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            resArr[i][j] = resArr[i][j] * det;
        }
    }

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

bool Matrix4::operator ==(const Matrix4& secondOperand) const {
    for (int i=0; i<4; i++){
        for (int j=0; j<4; j++){
            if (Matrix[i][j] != secondOperand.Matrix[i][j]){
                return false; // Quick reject
            }
        }
    }
    return true;
}

Matrix4& Matrix4::operator =(const Matrix4& secondOperand) {
    for (int i=0; i<4; i++){
        for (int j=0; j<4; j++){
            Matrix[i][j] = secondOperand.Matrix[i][j];
        }
    }
    return *this;
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