#pragma once
#define outputprecision 64
#include <fstream>
#include <iostream>

namespace EigenMatrixIO{
	template<class Matrix>
	void write_binary(const char* filename, const Matrix& matrix){
		std::ofstream out(filename, std::ios::out | std::ios::binary | std::ios::trunc);
		out.precision(outputprecision);
		typename Matrix::Index rows = matrix.rows(), cols = matrix.cols();
		out.write((char*)(&rows), sizeof(typename Matrix::Index));
		out.write((char*)(&cols), sizeof(typename Matrix::Index));
		out.write((char*)matrix.data(), rows*cols*sizeof(typename Matrix::Scalar));
		out.close();
	};

	template<class Matrix>
	void write_binary(std::ofstream & out, const Matrix& matrix){
		typename Matrix::Index rows = matrix.rows(), cols = matrix.cols();
		out.write((char*)(&rows), sizeof(typename Matrix::Index));
		out.write((char*)(&cols), sizeof(typename Matrix::Index));
		out.write((char*)matrix.data(), rows*cols*sizeof(typename Matrix::Scalar));
	};

	template<class Matrix>
	bool read_binary(const char* filename, Matrix& matrix){
		std::ifstream in(filename, std::ios::in | std::ios::binary);
		if (!in.good())
		{
			std::cout << "file not open" << std::endl;
			return false;
		}
		typename Matrix::Index rows = 0, cols = 0;
		in.read((char*)(&rows), sizeof(typename Matrix::Index));
		in.read((char*)(&cols), sizeof(typename Matrix::Index));
		matrix.resize(rows, cols);
		in.read((char *)matrix.data(), rows*cols*sizeof(typename Matrix::Scalar));
		in.close();
		return true;
	};

	template<class Matrix>
	void read_binary(std::ifstream &in, Matrix& matrix){
		/*if (!in.good())
		{
			std::cout << "file not open" << std::endl;
			return;
		}*/
		typename Matrix::Index rows = 0, cols = 0;
		in.read((char*)(&rows), sizeof(typename Matrix::Index));
		in.read((char*)(&cols), sizeof(typename Matrix::Index));
		matrix.resize(rows, cols);
		in.read((char *)matrix.data(), rows*cols*sizeof(typename Matrix::Scalar));
	};
};