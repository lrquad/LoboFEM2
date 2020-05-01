#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
namespace lobo
{
	template <class TYPE>
	int matrixNormalize(Eigen::SparseMatrix<TYPE> *matrix, Eigen::Matrix<TYPE,-1,1> *vector)
	{
		double result = 0;
		for (int j = 0; j < matrix->outerSize(); ++j)
			for (Eigen::SparseMatrix<TYPE>::InnerIterator it(*matrix, j); it; ++it)
			{
				int r = it.row(); int c = it.col();
				if (c < r)
				{
					continue;
				}

				if (c == r)
				{
					result += it.value()*vector->data()[r] * vector->data()[c];
				}
				else
				{
					result += 2.0*it.value()*vector->data()[r] * vector->data()[c];
				}
			}
		result = std::sqrt(result);
		*vector /= result;
		return 1;
	}



	template <class TYPE>
	int MatrixPCA(Eigen::Matrix<TYPE, -1, -1> *A, int r, SparseMatrix<TYPE> *weights)
	{
		int rows = A->rows();
		int cols = A->cols();

		Eigen::Matrix<TYPE, -1, -1> dataset(rows, cols);

		dataset = A->transpose();

		JacobiSVD<Eigen::Matrix<TYPE, -1, -1>> svd(dataset, ComputeThinU | ComputeThinV);

		dataset = svd.matrixV();

		A->resize(rows, r);
		for (int i = 0; i < r; i++)
		{
			A->col(i) = dataset.col(i);
		}
		//*A = dataset;
		return 1;
	}

}