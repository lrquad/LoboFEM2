#pragma once
#include <map>
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;

class SparseMatrixTopology
{
public:
	SparseMatrixTopology(SparseMatrix<double>* sparseMatrix);
	~SparseMatrixTopology();

	int getValueIndex(int row, int col);
protected:
	
	SparseMatrix<double> sparse_matrix;
	std::map<int, int> mapOfSparse ;
	std::map<std::vector<int>, int> mapOfSparsev2;
	int rows_;
	int cols_;

};

template <class TYPE>
class SparseMatrixTopologyTYPE
{
	typedef Eigen::Triplet<TYPE> EIGEN_TRI;

public:
	SparseMatrixTopologyTYPE(SparseMatrix<TYPE>* sparseMatrix);
	~SparseMatrixTopologyTYPE();

	int getValueIndex(int row, int col);

protected:

	SparseMatrix<TYPE> sparse_matrix;
	std::map<int, int> mapOfSparse;
	std::map<std::vector<int>, int> mapOfSparsev2;
	int rows_;
	int cols_;
};

template <class TYPE>
int SparseMatrixTopologyTYPE<TYPE>::getValueIndex(int row, int col)
{
	std::vector<int> rowcolindex(2);
	rowcolindex[0] = row;
	rowcolindex[1] = col;

	return mapOfSparsev2[rowcolindex];
}

template <class TYPE>
SparseMatrixTopologyTYPE<TYPE>::~SparseMatrixTopologyTYPE()
{

}

template <class TYPE>
SparseMatrixTopologyTYPE<TYPE>::SparseMatrixTopologyTYPE(SparseMatrix<TYPE>* sparseMatrix)
{
	rows_ = sparseMatrix->rows();
	cols_ = sparseMatrix->cols();

	this->sparse_matrix.resize(sparseMatrix->rows(), sparseMatrix->cols());
	std::vector<EIGEN_TRI> resultcoef;

	int i = 0;
	for (int j = 0; j < sparseMatrix->outerSize(); ++j)
		for (typename SparseMatrix<TYPE>::InnerIterator it(*sparseMatrix, j); it; ++it)
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

