
#ifndef _Matrix4_h_
#define _Matrix4_h_

class Matrix4 {

    public:
        // Constructor with and without initialisation array
        Matrix4();
        Matrix4(float arr[4][4]);
  
    public:
        // Get method for matrix
        float getElement(int rowindex, int columnindex) const { return Matrix[rowindex][columnindex]; };

        // Transpose
        Matrix4 transpose();

        // Operator Overloading
        const Matrix4 operator +(const Matrix4& secondOperand) const;
        const Matrix4 operator -(const Matrix4& secondOperand) const;
        const Matrix4 operator *(const Matrix4& secondOperand) const;

    private:
        // The actual Matrix: [rows][columns]
        float Matrix[4][4];
};

// Right and left multiplication with a scalar
const Matrix4 operator*(float scalar, const Matrix4& matrix);
const Matrix4 operator*(const Matrix4& matrix, float scalar);

#endif