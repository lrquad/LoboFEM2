#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;

double SparseMatrixGetMaxAbsEntry(SparseMatrix<double>* sparseMatrix)
{
	double norm0 = 0.0;
	int i = 0;
	for (int j = 0; j < sparseMatrix->outerSize(); ++j)
		for (SparseMatrix<double>::InnerIterator it(*sparseMatrix, j); it; ++it, i++)
		{
			if (fabs((double)it.value())>norm0)
			{
				norm0 = it.value();
			}
		}
	return norm0;
}