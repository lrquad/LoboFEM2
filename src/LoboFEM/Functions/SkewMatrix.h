#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;

void skewMatrix(Vector3d &w, Matrix3d &result);
void skewVector(Vector3d &result, Matrix3d &w);

void getRowMajorFromColMajor(Matrix3d &input, double* M);
void rowMajorToColMajor(double* M, Matrix3d &output);

Vector3d fromSkewSysmmtric(const Matrix3d &R);