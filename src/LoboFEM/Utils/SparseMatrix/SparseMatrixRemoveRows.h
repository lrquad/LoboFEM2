#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "SparseMatrixTopology.h"
#include <iostream>



void subSparseMatrix(SparseMatrix<double> &source, SparseMatrix<double> &result, std::vector<int> &map)
;

void SparseMatrixRemoveRows(SparseMatrix<double>* sparseMatrix, SparseMatrix<double>* resultMatrix, std::vector<int>& entry_map, int r, int numConstrainedDOFs_, int* constrainedDOFs_)
;

void VectorRemoveRows(std::vector<int> &map, VectorXd &target, VectorXd &result, int numConstrainedDOFs_ = NULL, int* constrainedDOFs_ = NULL)
;

void VectorInsertRows(std::vector<int> &map, VectorXd &target, VectorXd &result, int numConstrainedDOFs_ = NULL, int* constrainedDOFs_ = NULL)
;

void MatrixRemoveDofs(std::vector<int> &map, MatrixXd& target,MatrixXd &result);

void MatrixInsertDofs(std::vector<int> &map, MatrixXd& target, MatrixXd &result);

void MatrixRemoveRows(std::vector<int> &map, MatrixXd& target, MatrixXd &result);

void MatrixRemoveCols(std::vector<int> &map, MatrixXd& target, MatrixXd &result);

void createMapByConstrains(std::vector<int> &map, int r, int numConstrainedDOFs_, int* constrainedDOFs_);

/// <summary>
/// Creates the sparse map by topology.
/// </summary>
/// <param name="sparseMatrix">The sparse matrix.</param>
/// <param name="subsparseMatrix">The subsparse matrix.</param>
/// <param name="entry_map">The entry_map.</param>
/// <param name="rowmap">The rowmap.</param>
/// <param name="r">The r.</param>
/// <param name="numConstrainedDOFs_">The number constrained do FS_.</param>
/// <param name="constrainedDOFs_">The constrained do FS_.</param>
void createSparseMapbyTopology(SparseMatrix<double>* sparseMatrix, SparseMatrix<double>* subsparseMatrix, std::vector<int>& entry_map, std::vector<int>& rowmap, int r, int numConstrainedDOFs_, int* constrainedDOFs_);



namespace lobo
{


#define VectorX_INPUT Eigen::Matrix<TYPE, -1, 1>

	template <class TYPE>
	void subSparseMatrix(SparseMatrix<TYPE> &source, SparseMatrix<TYPE> &result, std::vector<int> &map)
	{
		std::vector<Eigen::Triplet<TYPE>> reusltcoef;
		if (map.size() == source.rows())
		{
			for (int j = 0; j < source.outerSize(); ++j)
				for (typename SparseMatrix<TYPE>::InnerIterator it(source, j); it; ++it)
				{
					int i_p = map[it.row()];
					int j_p = map[it.col()];
					if (!(i_p == -1 || j_p == -1))
					{
						reusltcoef.push_back(Eigen::Triplet<TYPE>(i_p, j_p, it.value()));
					}
				}
			result.setFromTriplets(reusltcoef.begin(), reusltcoef.end());
		}
		else
			if (map.size() == result.rows())
			{
				std::cout << "sparse matrix does not support this map" << std::endl;
				return;
			}
	}

	template <class TYPE>
	void subSparseMatrixdouble(SparseMatrix<TYPE> &source, SparseMatrix<double> &result, std::vector<int> &map)
	{
		std::vector<Eigen::Triplet<double>> reusltcoef;
		if (map.size() == source.rows())
		{
			for (int j = 0; j < source.outerSize(); ++j)
				for (typename SparseMatrix<TYPE>::InnerIterator it(source, j); it; ++it)
				{
					int i_p = map[it.row()];
					int j_p = map[it.col()];
					if (!(i_p == -1 || j_p == -1))
					{
						reusltcoef.push_back(Eigen::Triplet<double>(i_p, j_p, (double)it.value()));
					}
				}
			result.setFromTriplets(reusltcoef.begin(), reusltcoef.end());
		}
		else
			if (map.size() == result.rows())
			{
				std::cout << "sparse matrix does not support this map" << std::endl;
				return;
			}
	}


	template <class TYPE>
	void createSparseMapbyTopology(SparseMatrix<TYPE>* sparseMatrix, SparseMatrix<TYPE>* subsparseMatrix, std::vector<int>& entry_map, std::vector<int>& rowmap, int r, int numConstrainedDOFs_, int* constrainedDOFs_)
	{
		subsparseMatrix->resize(r - numConstrainedDOFs_, r - numConstrainedDOFs_);
		std::vector<Triplet<TYPE>> subcoef;

		for (int j = 0; j < sparseMatrix->outerSize(); ++j)
			for (typename SparseMatrix<TYPE>::InnerIterator it(*sparseMatrix, j); it; ++it)
			{
				int i_p = rowmap[it.row()];
				int j_p = rowmap[it.col()];
				if (!(i_p == -1 || j_p == -1))
				{
					subcoef.push_back(Triplet<TYPE>(i_p, j_p, it.value()));
				}
			}
		subsparseMatrix->setFromTriplets(subcoef.begin(), subcoef.end());

		int supersize = sparseMatrix->nonZeros();
		int subsize = subsparseMatrix->nonZeros();

		entry_map.resize(supersize);
		std::fill(entry_map.begin(), entry_map.end(), -1);

		SparseMatrixTopologyTYPE<TYPE> supermatrix(sparseMatrix);
		SparseMatrixTopologyTYPE<TYPE> submatrix(subsparseMatrix);

		for (int j = 0; j < sparseMatrix->outerSize(); ++j)
			for (typename SparseMatrix<TYPE>::InnerIterator it(*sparseMatrix, j); it; ++it)
			{
				int i_p = rowmap[it.row()];
				int j_p = rowmap[it.col()];
				if (!(i_p == -1 || j_p == -1))
				{
					entry_map[supermatrix.getValueIndex(it.row(), it.col())] =
						submatrix.getValueIndex(i_p, j_p);
				}
			}
	}

	template <class TYPE>
	void SparseMatrixRemoveRows(SparseMatrix<TYPE>* sparseMatrix, SparseMatrix<TYPE>* resultMatrix, std::vector<int>& entry_map, int r, int numConstrainedDOFs_, int* constrainedDOFs_)
	{
		int i = 0;
		for (int j = 0; j < sparseMatrix->outerSize(); ++j)
			for (typename SparseMatrix<TYPE>::InnerIterator it(*sparseMatrix, j); it; ++it, ++i)
			{
				if (entry_map[i] != -1)
				{
					resultMatrix->valuePtr()[entry_map[i]] = it.value();
				}
			}
	}

	template <class TYPE>
	void VectorRemoveRows(std::vector<int> &map, VectorX_INPUT &target, VectorX_INPUT &result, int numConstrainedDOFs_ /*= NULL*/, int* constrainedDOFs_ /*= NULL*/)
	{
		for (int i = 0; i < target.size(); i++)
		{
			if (map[i] != -1)
			{
				result.data()[map[i]] = target.data()[i];
			}
		}
	}

	template <class TYPE>
	void VectorInsertRows(std::vector<int> &map, VectorX_INPUT &target, VectorX_INPUT &result, int numConstrainedDOFs_, int* constrainedDOFs_)
	{
		result.setZero();
		for (int i = 0; i < result.size(); i++)
		{
			if (map[i] != -1)
			{
				result.data()[i] = target.data()[map[i]];
			}
		}
	}


	template <class TYPE>
	void VectorInsertRows(std::vector<int> &map, VectorX_INPUT &target, VectorX_INPUT &result)
	{
		result.setZero();
		for (int i = 0; i < result.size(); i++)
		{
			if (map[i] != -1)
			{
				result.data()[i] = target.data()[map[i]];
			}
		}
	}

}