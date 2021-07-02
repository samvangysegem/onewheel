#ifndef _Vector4_h_
#define _Vector4_h_

#include "Matrix4.h"

class RowVector4 {

    public:
        // Constructor with and without initialisation array
        RowVector4();
        RowVector4(float arr[4]);
  
    public:
        // Get method for matrix
        float getElement(int index) const { return Vector[index]; };

        // Operator Overloading
        const RowVector4 operator +(const RowVector4& secondOperand) const;
        const RowVector4 operator -(const RowVector4& secondOperand) const;

    private:
        // The actual Matrix: [rows][columns]
        float Vector[4];
};

class ColumnVector4 {

    public:
        // Constructor with and without initialisation array
        ColumnVector4();
        ColumnVector4(float arr[4]);
  
    public:
        // Get method for matrix
        float getElement(int index) const { return Vector[index]; };

        // Operator Overloading
        const ColumnVector4 operator +(const ColumnVector4& secondOperand) const;
        const ColumnVector4 operator -(const ColumnVector4& secondOperand) const;

    private:
        // The actual Matrix: [rows][columns]
        float Vector[4];
};

// Right and left multiplication with a scalar
const RowVector4 operator*(float scalar, const RowVector4& vec);
const RowVector4 operator*(const RowVector4& vec, float scalar);
const ColumnVector4 operator*(float scalar, const ColumnVector4& vec);
const ColumnVector4 operator*(const ColumnVector4& vec, float scalar);

// Vector and Matrix multiplication overloading
const float operator*(const RowVector4& firstOperand, const ColumnVector4& secondOperand);
const Matrix4 operator*(const ColumnVector4& firstOperand, const RowVector4& secondOperand);
const RowVector4 operator*(const RowVector4& firstOperand, const Matrix4& secondOperand);
const ColumnVector4 operator*(const Matrix4& firstOperand, const ColumnVector4& secondOperand);

#endif