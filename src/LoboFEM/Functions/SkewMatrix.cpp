#include "SkewMatrix.h"

void skewMatrix(Vector3d &w, Matrix3d &result)
{
	result.setZero();
	result.data()[1] = w.data()[2];
	result.data()[2] = -w.data()[1];
	result.data()[5] = w.data()[0];
	Matrix3d temp = result.transpose();
	result -= temp;
}

void skewVector(Vector3d &result, Matrix3d &w)
{
	Matrix3d A = (w - w.transpose()) / 2;
	result.setZero();
	result.data()[0] = A.data()[5];
	result.data()[1] = -A.data()[2];
	result.data()[2] = A.data()[1];
}

void getRowMajorFromColMajor(Matrix3d &input, double* M)
{
	M[0] = input.data()[0];
	M[1] = input.data()[3];
	M[2] = input.data()[6];
	M[3] = input.data()[1];
	M[4] = input.data()[4];
	M[5] = input.data()[7];
	M[6] = input.data()[2];
	M[7] = input.data()[5];
	M[8] = input.data()[8];
}

void rowMajorToColMajor(double* M, Matrix3d &output)
{
	output.data()[0] = M[0];
	output.data()[1] = M[0 + 3];
	output.data()[2] = M[0 + 6];
	output.data()[3] = M[1];
	output.data()[4] = M[1 + 3];
	output.data()[5] = M[1 + 6];
	output.data()[6] = M[2 + 0];
	output.data()[7] = M[2 + 3];
	output.data()[8] = M[2 + 6];
}

Vector3d fromSkewSysmmtric(const Matrix3d &R)
{
	Vector3d temp;
	temp.data()[0] = R.data()[5];
	temp.data()[1] = R.data()[6];
	temp.data()[2] = R.data()[1];
	return temp;
}
