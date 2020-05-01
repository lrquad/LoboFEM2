#include "SparseMatrixTopology.h"
#include <iostream>
typedef Eigen::Triplet<double> EIGEN_TRI;

SparseMatrixTopology::SparseMatrixTopology(SparseMatrix<double>* sparseMatrix)
{
	rows_ = sparseMatrix->rows();
	cols_ = sparseMatrix->cols();
	//copy sparseMatrix topology and set all value to 0 first
	this->sparse_matrix.resize(sparseMatrix->rows(), sparseMatrix->cols());
	std::vector<EIGEN_TRI> resultcoef;
	int i = 0;
	for (int j = 0; j < sparseMatrix->outerSize(); ++j)
		for (SparseMatrix<double>::InnerIterator it(*sparseMatrix, j); it; ++it)
		{
			resultcoef.push_back(EIGEN_TRI(it.row(), it.col(), 0));
			std::vector<int> rowcolindex(2);
			rowcolindex[0] = it.row();
			rowcolindex[1] = it.col();
			mapOfSparsev2.insert(std::make_pair(rowcolindex, i));
			//mapOfSparse.insert(std::make_pair(it.col()*sparseMatrix->rows() + it.row(), i));
			/*if (it.col() == 291456 && it.row() == 291456)
			{
				std::cout << "map index" << std::endl;
				std::cout << i << std::endl;
			}*/

			i++;
		}
	sparse_matrix.setFromTriplets(resultcoef.begin(), resultcoef.end());
}


SparseMatrixTopology::~SparseMatrixTopology()
{

}

int SparseMatrixTopology::getValueIndex(int row, int col)
{
	std::vector<int> rowcolindex(2);
	rowcolindex[0] = row;
	rowcolindex[1] = col;

	return mapOfSparsev2[rowcolindex];

	int i = 0;
	
	for (int j = 0; j < sparse_matrix.outerSize(); j++)
	{
		for (SparseMatrix<double>::InnerIterator it(sparse_matrix, j); it; ++it,i++)
		{
			if (row == it.row() && col == it.col())
			{
				return i;
			}
		}
	}

	return -1;
}