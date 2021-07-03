#ifndef _Vector4_h_
#define _Vector4_h_

#include "Matrix4.h"

class ColumnVector4;

class RowVector4 {

    public:
        // Constructor with and without initialisation array
        RowVector4();
        RowVector4(float arr[4]);
  
    public:
        // Get method for matrix
        const ColumnVector4 T();
        float getElement(int index) const { return Vector[index]; };
        void setElement(int index, float val) { Vector[index] = val; };

        // Operator Overloading
        const RowVector4 operator +(const RowVector4& secondOperand) const;
        const RowVector4 operator -(const RowVector4& secondOperand) const;
        bool operator ==(const RowVector4& secondOperand) const;
        RowVector4& operator =(const RowVector4& secondOperand);

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
        const RowVector4 T();
        float getElement(int index) const { return Vector[index]; };
        void setElement(int index, float val) { Vector[index] = val; };

        // Operator Overloading
        const ColumnVector4 operator +(const ColumnVector4& secondOperand) const;
        const ColumnVector4 operator -(const ColumnVector4& secondOperand) const;
        bool operator ==(const ColumnVector4& secondOperand) const;
        ColumnVector4& operator =(const ColumnVector4& secondOperand);

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