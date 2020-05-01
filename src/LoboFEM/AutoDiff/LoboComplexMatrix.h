#pragma once
#include "AutoDiff/LoboComplex.h"
#include <vector>

#define CUBIC2 (TYPE)1.25992104989487316476721061
//2^(1/3)
#define CUBIC23 (TYPE)1.58740105196819947475170564 
//2^(2/3)

namespace lobo {

	template<class COMPLEX_TYPE, class TYPE> class LoboComplexMatrix;
	template<class COMPLEX_TYPE, class TYPE> class LoboComplexMatrix3;


#define LoboComplexMatrixTYPE LoboComplexMatrix<COMPLEX_TYPE,TYPE>
#define LoboComplexMatrixTYPE3 LoboComplexMatrix3<COMPLEX_TYPE,TYPE>

	template<class COMPLEX_TYPE, class TYPE>
	COMPLEX_TYPE inner_product(const LoboComplexMatrixTYPE3& a, const LoboComplexMatrixTYPE3& b);
	/*template<class COMPLEX_TYPE, class TYPE>
	void multi(const LoboComplexMatrixTYPE& a, const LoboComplexMatrixTYPE& b, LoboComplexMatrixTYPE& c);*/
	

	template<class COMPLEX_TYPE, class TYPE>
	class LoboComplexMatrix {

	public:
		LoboComplexMatrix(int row, int col)
		{
			row_ = row;
			col_ = col;
			data.resize(row_*col_);
			size_ = row_*col_;
		}

		LoboComplexMatrix(const LoboComplexMatrix<COMPLEX_TYPE, TYPE>& other){
			data = other.data;
		};

		LoboComplexMatrix():row_(0),col_(0),size_(0)
		{

		}

		~LoboComplexMatrix()
		{

		}

		virtual void setZero();
		virtual void setIdentity();
		virtual void setRandom();


		virtual COMPLEX_TYPE trace();


		std::vector<COMPLEX_TYPE> data;
		int row_, col_;
		int size_;

	};

	template<class COMPLEX_TYPE,class TYPE>
	class LoboComplexMatrix3:public LoboComplexMatrix<COMPLEX_TYPE,TYPE>{
	public:
		using LoboComplexMatrix<COMPLEX_TYPE,TYPE>::data;

		LoboComplexMatrix3() :LoboComplexMatrix<COMPLEX_TYPE, TYPE>(3, 3) {};
		LoboComplexMatrix3(const LoboComplexMatrix<COMPLEX_TYPE, TYPE>& other) :LoboComplexMatrix<COMPLEX_TYPE, TYPE>(3, 3) {
			data = other.data;
		};

		virtual COMPLEX_TYPE det();
		virtual COMPLEX_TYPE norm_2();
		virtual void setRealValue(const TYPE* matrix3d);
		virtual LoboComplexMatrixTYPE3 transpose();
		virtual LoboComplexMatrixTYPE3 inverse();
		virtual COMPLEX_TYPE sumdiagAdjoint();

		virtual void computeEigenVectors( LoboComplexMatrixTYPE3& eigenvectors, COMPLEX_TYPE* eigenvalues);

		virtual void computeSVD_V(LoboComplexMatrixTYPE3& V, COMPLEX_TYPE* eigenvalues);
	};

	
	
	

	template<class COMPLEX_TYPE, class TYPE>
	void lobo::LoboComplexMatrix3<COMPLEX_TYPE, TYPE>::setRealValue(const TYPE* matrix3d)
	{
		for (int i = 0;i < 9;i++)
		{
			data[i] = matrix3d[i];
		}
	}
	

	template<class COMPLEX_TYPE, class TYPE>
	inline LoboComplexMatrix<COMPLEX_TYPE, TYPE> operator*(const LoboComplexMatrix<COMPLEX_TYPE, TYPE>& lhs, const LoboComplexMatrix<COMPLEX_TYPE, TYPE>& rhs)
	{
		int row = lhs.row_;
		int col = rhs.col_;
		LoboComplexMatrix<COMPLEX_TYPE, TYPE> result(row, col);
		COMPLEX_TYPE tmp;
		result.setZero();

		for (int i = 0;i < row;i++)
		{
			for (int j = 0;j < col;j++)
			{
				for (int k = 0;k < lhs.col_;k++)
				{
					//lobo::multi<COMPLEX_TYPE,TYPE>(lhs.data[k*lhs.row_ + i], rhs.data[j*rhs.row_ + k], tmp);
					result.data[j*row + i] += lhs.data[k*lhs.row_ + i] * rhs.data[j*rhs.row_ + k];
					//lobo::add_a(result.data[j*row + i], tmp);
				}
			}
		}
		return result;
	}

	template<class COMPLEX_TYPE, class TYPE>
	inline void multiM(const LoboComplexMatrixTYPE& lhs, const LoboComplexMatrixTYPE& rhs, LoboComplexMatrixTYPE& result)
	{
		int row = lhs.row_;
		int col = rhs.col_;
		//LoboComplexMatrix<COMPLEX_TYPE, TYPE> result(row, col);
		COMPLEX_TYPE tmp;
		result.setZero();

		for (int i = 0;i < row;i++)
		{
			for (int j = 0;j < col;j++)
			{
				for (int k = 0;k < lhs.col_;k++)
				{
					lobo::multi<COMPLEX_TYPE,TYPE>(lhs.data[k*lhs.row_ + i], rhs.data[j*rhs.row_ + k], tmp);
					//result.data[j*row + i] += lhs.data[k*lhs.row_ + i] * rhs.data[j*rhs.row_ + k];
					lobo::add_a(result.data[j*row + i], tmp);
				}
			}
		}
	}



	template<class COMPLEX_TYPE, class TYPE>
	void lobo::LoboComplexMatrixTYPE::setRandom()
	{
		for (int i = 0;i < size_;i++)
		{
			data[i].real_ = (TYPE)std::rand()/ RAND_MAX;
			data[i].image_ = (TYPE)std::rand()/ RAND_MAX;
		}
	}

	template<class COMPLEX_TYPE, class TYPE>
	COMPLEX_TYPE lobo::LoboComplexMatrixTYPE::trace()
	{
		int min_r = std::min(row_, col_);
		COMPLEX_TYPE result;
		
		for (int i = 0;i < min_r;i++)
		{
			result += data[i*row_ + i];
		}
		return result;
	}

	template<class COMPLEX_TYPE, class TYPE>
	void lobo::LoboComplexMatrixTYPE::setIdentity()
	{
		int min_r = std::min(row_, col_);
		for (int i = 0;i < min_r;i++)
		{
			data[i*row_ + i].real_ = (TYPE)1.0;
			data[i*row_ + i].image_ = (TYPE)0.0;
		}
	}

	template<class COMPLEX_TYPE, class TYPE>
	void lobo::LoboComplexMatrixTYPE::setZero()
	{
		memset(data.data(), 0, sizeof(COMPLEX_TYPE)*row_*col_);
	}

	template<class COMPLEX_TYPE, class TYPE>
	std::ostream& operator<<(std::ostream& os, const LoboComplexMatrixTYPE& __x)
	{
		for (int i = 0;i < __x.row_;i++)
		{
			for (int j = 0;j < __x.col_;j++)
			{
				os << __x.data[j*__x.row_ + i] << " ";
			}
			os << std::endl;
		}
		
		return os;
	}


	template<class COMPLEX_TYPE, class TYPE>
	inline LoboComplexMatrixTYPE operator*(const LoboComplexMatrixTYPE& lhs, const LoboComplexMatrixTYPE* rhs)
	{
		LoboComplexMatrixTYPE result;
		result.setZero();
		for (int i = 0;i < lhs.row_;i++)
		{
			for (int j = 0;j < rhs.col_;j++)
			{
				for (int k = 0;k < lhs.col_;k++)
				{
					result.data[j * 3 + i] += lhs.data[k * 3 + i] * rhs[j * 3 + k];
				}
			}
		}
		return result;
	}

	
	//3x3 matrix

	template<class COMPLEX_TYPE, class TYPE>
	inline LoboComplexMatrixTYPE3 operator*(const LoboComplexMatrixTYPE3& lhs, const LoboComplexMatrixTYPE3& rhs)
	{
		LoboComplexMatrixTYPE3 result;
		COMPLEX_TYPE tmp;
		result.setZero();
		for (int i = 0;i < 3;i++)
		{
			for (int j = 0;j < 3;j++)
			{
				for (int k = 0;k < 3;k++)
				{
					result.data[j * 3 + i] += lhs.data[k * 3 + i] * rhs.data[j * 3 + k];
					//lobo::multi<COMPLEX_TYPE, TYPE>(lhs.data[k * 3 + i], rhs.data[j * 3 + k], tmp);
					//result.data[j*row + i] += lhs.data[k*lhs.row_ + i] * rhs.data[j*rhs.row_ + k];
					//lobo::add_a(result.data[j * 3 + i], tmp);
				}
			}
		}
		return result;
	}

	template<class COMPLEX_TYPE, class TYPE>
	inline void multiM(const LoboComplexMatrixTYPE3& lhs, const LoboComplexMatrixTYPE3& rhs, LoboComplexMatrixTYPE3& result)
	{
		//LoboComplexMatrix<COMPLEX_TYPE, TYPE> result(row, col);
		COMPLEX_TYPE tmp;
		result.setZero();

		for (int i = 0;i < 3;i++)
		{
			for (int j = 0;j < 3;j++)
			{
				for (int k = 0;k < 3;k++)
				{
					lobo::multi<COMPLEX_TYPE, TYPE>(lhs.data[k*3 + i], rhs.data[j*3 + k], tmp);
					//result.data[j*row + i] += lhs.data[k*lhs.row_ + i] * rhs.data[j*rhs.row_ + k];
					lobo::add_a(result.data[j*3 + i], tmp);
				}
			}
		}
	}

	//lhsT* rhs
	template<class COMPLEX_TYPE, class TYPE>
	inline void multiMT(const LoboComplexMatrixTYPE3& lhs, const LoboComplexMatrixTYPE3& rhs, LoboComplexMatrixTYPE3& result)
	{
		//LoboComplexMatrix<COMPLEX_TYPE, TYPE> result(row, col);
		COMPLEX_TYPE tmp;
		result.setZero();

		for (int i = 0;i < 3;i++)
		{
			for (int j = 0;j < 3;j++)
			{
				for (int k = 0;k < 3;k++)
				{
					lobo::multi<COMPLEX_TYPE, TYPE>(lhs.data[i * 3 + k], rhs.data[j * 3 + k], tmp);
					//result.data[j*row + i] += lhs.data[k*lhs.row_ + i] * rhs.data[j*rhs.row_ + k];
					lobo::add_a(result.data[j * 3 + i], tmp);
				}
			}
		}
	}

	//For Matrix*EigenDense
	template<class COMPLEX_TYPE, class TYPE>
	inline LoboComplexMatrixTYPE3 operator*(const LoboComplexMatrixTYPE3& lhs, const TYPE* rhs)
	{
		LoboComplexMatrixTYPE3 result;
		result.setZero();
		for (int i = 0;i < 3;i++)
		{
			for (int j = 0;j < 3;j++)
			{
				for (int k = 0;k < 3;k++)
				{
					result.data[j * 3 + i] += lhs.data[k * 3 + i] * rhs[j * 3 + k];
				}
			}
		}
		return result;
	}

	template<class COMPLEX_TYPE, class TYPE>
	inline LoboComplexMatrixTYPE3 operator*(const LoboComplexMatrixTYPE3& lhs, const TYPE& rhs)
	{
		LoboComplexMatrixTYPE3 result;
		result.setZero();
		
		for (int i = 0;i < 9;i++)
		{
			result.data[i] = lhs.data[i] * rhs;
		}

		return result;
	}

	template<class COMPLEX_TYPE, class TYPE>
	inline void multiMScalar(const LoboComplexMatrixTYPE3& lhs, const TYPE& rhs, LoboComplexMatrixTYPE3& result)
	{
		//LoboComplexMatrix<COMPLEX_TYPE, TYPE> result(row, col);
		result.setZero();
		for (int i = 0;i < 9;i++)
		{
			result.data[i] = lhs.data[i] * rhs;
		}
	}


	template<class COMPLEX_TYPE, class TYPE>
	inline LoboComplexMatrixTYPE3 operator+(const LoboComplexMatrixTYPE3& lhs, const LoboComplexMatrixTYPE3& rhs)
	{
		LoboComplexMatrixTYPE3 result;
		result.setZero();
		for (int i = 0;i < 3;i++)
		{
			for (int j = 0;j < 3;j++)
			{

				result.data[j * 3 + i] = lhs.data[j * 3 + i] + rhs.data[j * 3 + i];
			}
		}
		return result;
	}

	template<class COMPLEX_TYPE, class TYPE>
	inline void addM(const LoboComplexMatrixTYPE3& lhs, const LoboComplexMatrixTYPE3& rhs, LoboComplexMatrixTYPE3& result)
	{
		//LoboComplexMatrix<COMPLEX_TYPE, TYPE> result(row, col);
		result.setZero();
		for (int i = 0;i < 3;i++)
		{
			for (int j = 0;j < 3;j++)
			{
				//result.data[j * 3 + i] = lhs.data[j * 3 + i] + rhs.data[j * 3 + i];
				lobo::add<COMPLEX_TYPE, TYPE>(lhs.data[j * 3 + i], rhs.data[j * 3 + i], result.data[j * 3 + i]);
			}
		}
	}

	template<class COMPLEX_TYPE, class TYPE>
	inline void addM_a(LoboComplexMatrixTYPE3& lhs, const LoboComplexMatrixTYPE3& rhs)
	{
		for (int i = 0;i < 3;i++)
		{
			for (int j = 0;j < 3;j++)
			{
				//result.data[j * 3 + i] = lhs.data[j * 3 + i] + rhs.data[j * 3 + i];
				lobo::add_a(lhs.data[j * 3 + i], rhs.data[j * 3 + i]);
			}
		}
	}


	//for eigen
	template<class COMPLEX_TYPE, class TYPE>
	inline LoboComplexMatrixTYPE3 operator+(const LoboComplexMatrixTYPE3& lhs, const TYPE* rhs)
	{
		LoboComplexMatrixTYPE3 result;
		result.setZero();
		for (int i = 0;i < 3;i++)
		{
			for (int j = 0;j < 3;j++)
			{

				result.data[j * 3 + i] = lhs.data[j * 3 + i] + rhs[j * 3 + i];
			}
		}
		return result;
	}


	template<class COMPLEX_TYPE, class TYPE>
	inline LoboComplexMatrixTYPE3 operator-(const LoboComplexMatrixTYPE3& lhs, const LoboComplexMatrixTYPE3& rhs)
	{
		LoboComplexMatrixTYPE3 result;
		result.setZero();
		for (int i = 0;i < 3;i++)
		{
			for (int j = 0;j < 3;j++)
			{

				result.data[j * 3 + i] = lhs.data[j * 3 + i] - rhs.data[j * 3 + i];
			}
		}
		return result;
	}


	template<class COMPLEX_TYPE, class TYPE>
	COMPLEX_TYPE lobo::LoboComplexMatrixTYPE3::det()
	{
		COMPLEX_TYPE det_;
		COMPLEX_TYPE tmp;
		COMPLEX_TYPE tmp2;
		lobo::multi<COMPLEX_TYPE, TYPE>(data[4], data[8], tmp);
		lobo::multi<COMPLEX_TYPE, TYPE>(data[5], data[7], tmp2);
		tmp -= tmp2;
		lobo::multi<COMPLEX_TYPE, TYPE>(data[0], tmp, det_);

		lobo::multi<COMPLEX_TYPE, TYPE>(data[1], data[8], tmp);
		lobo::multi<COMPLEX_TYPE, TYPE>(data[2], data[7], tmp2);
		tmp -= tmp2;
		lobo::multi<COMPLEX_TYPE, TYPE>(data[3], tmp, tmp2);
		det_ -= tmp2;

		lobo::multi<COMPLEX_TYPE, TYPE>(data[1], data[5], tmp);
		lobo::multi<COMPLEX_TYPE, TYPE>(data[2], data[4], tmp2);
		tmp -= tmp2;
		lobo::multi<COMPLEX_TYPE, TYPE>(data[6], tmp, tmp2);
		det_ += tmp2;


		/*det_ = data[0] * (data[4] * data[8] - data[5] * data[7])
			- data[3] * (data[1] * data[8] - data[2] * data[7])
			+ data[6] * (data[1] * data[5] - data[2] * data[4]);*/

		return det_;
	}

	template<class COMPLEX_TYPE, class TYPE>
	COMPLEX_TYPE lobo::LoboComplexMatrixTYPE3::sumdiagAdjoint()
	{
		COMPLEX_TYPE sum_;
		//sum_ *= 0;
		COMPLEX_TYPE tmp;
		lobo::multi<COMPLEX_TYPE,TYPE>(data[0], data[4], sum_);

		lobo::multi<COMPLEX_TYPE, TYPE>(data[1], data[3], tmp);
		sum_ -= tmp;
		lobo::multi<COMPLEX_TYPE, TYPE>(data[2], data[6], tmp);
		sum_ -= tmp;
		lobo::multi<COMPLEX_TYPE, TYPE>(data[5], data[7], tmp);
		sum_ -= tmp;
		lobo::multi<COMPLEX_TYPE, TYPE>(data[0], data[8], tmp);
		sum_ += tmp;
		lobo::multi<COMPLEX_TYPE, TYPE>(data[4], data[8], tmp);
		sum_ += tmp;

	/*	sum_ = -data[1] * data[3] + data[0] * data[4] - data[2] * data[6] - data[5] * data[7] + data[0] * data[8] + data[4] * data[8];*/
		return sum_;
	}


	template<class COMPLEX_TYPE, class TYPE>
	COMPLEX_TYPE lobo::LoboComplexMatrix3<COMPLEX_TYPE, TYPE>::norm_2()
	{
		COMPLEX_TYPE norm_;
		//norm_ *= 0;

		COMPLEX_TYPE tmp;
		for (int i = 0;i < 9;i++)
		{
			lobo::multi<COMPLEX_TYPE, TYPE>(data[i], data[i], tmp);
			//norm_ += data[i] * data[i];
			norm_ += tmp;
		}
		return norm_;
	}

	template<class COMPLEX_TYPE, class TYPE>
	COMPLEX_TYPE inner_product(const LoboComplexMatrixTYPE3& a, const LoboComplexMatrixTYPE3& b)
	{
		COMPLEX_TYPE result;
		COMPLEX_TYPE tmp;
		for (int i = 0;i < 9;i++)
		{
			lobo::multi<COMPLEX_TYPE, TYPE>(a.data[i], b.data[i], tmp);
			//norm_ += data[i] * data[i];
			result += tmp;
			//result += a.data[i] * b.data[i];
		}
		return result;
	}

	template<class COMPLEX_TYPE, class TYPE>
	LoboComplexMatrixTYPE3 lobo::LoboComplexMatrix3<COMPLEX_TYPE, TYPE>::transpose()
	{
		LoboComplexMatrixTYPE3 result;
		result.setZero();

		for (int i = 0;i < 3;i++)
		{
			for (int j = 0;j < 3;j++)
			{
				result.data[j * 3 + i] = data[i * 3 + j];
			}
		}
		return result;
	}

	template<class COMPLEX_TYPE, class TYPE>
	LoboComplexMatrixTYPE3 lobo::LoboComplexMatrix3<COMPLEX_TYPE, TYPE>::inverse()
	{
		LoboComplexMatrixTYPE3 result;
		result.setZero();
		COMPLEX_TYPE det_ = this->det();
		result.data[0] = -data[5] * data[7] + data[4] * data[8];
		result.data[1] = data[2] * data[7] - data[1] * data[8];
		result.data[2] = -data[2] * data[4] + data[1] * data[5];

		result.data[3] = data[5] * data[6] - data[3] * data[8];
		result.data[4] = -data[2] * data[6] + data[0] * data[8];
		result.data[5] = data[2] * data[3] - data[0] * data[5];

		result.data[6] = -data[4] * data[6] + data[3] * data[7];
		result.data[7] = data[1] * data[6] - data[0] * data[7];
		result.data[8] = -data[1] * data[3] + data[0] * data[4];

		for (int i = 0;i < 9;i++)
			result.data[i] /= det_;
		return result;
	}


	template<class COMPLEX_TYPE, class TYPE>
	void lobo::LoboComplexMatrix3<COMPLEX_TYPE, TYPE>::computeSVD_V(LoboComplexMatrixTYPE3& V, COMPLEX_TYPE* eigenvalues)
	{
		LoboComplexMatrixTYPE3 MTM;
		lobo::multiMT(*this, *this, MTM);

		MTM.computeEigenVectors( V, eigenvalues);

		/*COMPLEX_TYPE vdet = V.det();

		if (smallerReal(vdet, (TYPE)0.0))
		{
			V.data[0] *= (TYPE)-1.0;
			V.data[3] *= (TYPE)-1.0;
			V.data[6] *= (TYPE)-1.0;
		}*/

		COMPLEX_TYPE tmp;
		lobo::powS<COMPLEX_TYPE, TYPE>(eigenvalues[0], (TYPE)0.5, tmp);

		eigenvalues[0] = tmp;
		lobo::powS<COMPLEX_TYPE, TYPE>(eigenvalues[1], (TYPE)0.5, tmp);

		eigenvalues[1] = tmp;
		lobo::powS<COMPLEX_TYPE, TYPE>(eigenvalues[2], (TYPE)0.5, tmp);

		eigenvalues[2] = tmp;
		/*eigenvalues[0] = lobo::pow(eigenvalues[0], (TYPE)0.5);
		eigenvalues[1] = lobo::pow(eigenvalues[1], (TYPE)0.5);
		eigenvalues[2] = lobo::pow(eigenvalues[2], (TYPE)0.5);*/
	}

	template<class COMPLEX_TYPE, class TYPE>
	void lobo::LoboComplexMatrix3<COMPLEX_TYPE, TYPE>::computeEigenVectors(LoboComplexMatrixTYPE3& result, COMPLEX_TYPE* eigenvalues)
	{
		COMPLEX_TYPE det_ = this->det();
		COMPLEX_TYPE sumadj = this->sumdiagAdjoint();
		COMPLEX_TYPE trace_ = this->trace();
		COMPLEX_TYPE tmp_value,tmp_value2,tmp_value3,tmp_value4;

		//result.setZero();

		COMPLEX_TYPE term0 = (TYPE)4.0*sumadj*sumadj*sumadj + (TYPE)27.0*det_*det_ - (TYPE)18.0*sumadj*det_*trace_ - sumadj*sumadj*trace_*trace_ + (TYPE)4 * det_*trace_*trace_*trace_;

		COMPLEX_TYPE term1 = (TYPE)27.0*det_ - (TYPE)9 * sumadj*trace_ + (TYPE)2 * trace_*trace_*trace_
			/*+ (TYPE)(3.0 * std::sqrt(3.0))*lobo::pow(term0, (TYPE)(1.0 / 2.0))*/;

		COMPLEX_TYPE term1_complex = (TYPE)(3.0 * std::sqrt(3.0))*lobo::pow(-term0, (TYPE)(1.0 / 2.0));
		LoboComplex<COMPLEX_TYPE, double> dual_term1_complex;
		dual_term1_complex.real_ = term1;
		dual_term1_complex.image_ = term1_complex;

		COMPLEX_TYPE term2 = (TYPE)3 * sumadj - trace_*trace_;

		COMPLEX_TYPE term3 = lobo::pow(term1, (TYPE)(1.0 / 3.0));
		LoboComplex<COMPLEX_TYPE, double> dual_term3_complex = lobo::pow(dual_term1_complex, (TYPE)(1.0 / 3.0));
		

		COMPLEX_TYPE sqrt3;
		sqrt3.real_ = (TYPE)1.0;
		sqrt3.image_ = (TYPE)std::sqrt(3.0);
		COMPLEX_TYPE nsqrt3;
		nsqrt3.real_ = (TYPE)1.0;
		nsqrt3.image_ = -(TYPE)std::sqrt(3.0);
		COMPLEX_TYPE root1;
		COMPLEX_TYPE root2;
		COMPLEX_TYPE root3;

		LoboComplex<COMPLEX_TYPE, double> sqrt3_;
		LoboComplex<COMPLEX_TYPE, double> nsqrt3_;
		sqrt3_.real_ = (TYPE)1.0;
		sqrt3_.image_ = (TYPE)std::sqrt(3.0);

		nsqrt3_.real_ = (TYPE)1.0;
		nsqrt3_.image_ = -(TYPE)std::sqrt(3.0);


		//term2 / term3
		LoboComplex<COMPLEX_TYPE, double> term2dterm3;
		LoboComplex<COMPLEX_TYPE, double> dual_term2_complex;
		dual_term2_complex.real_ = term2;
		term2dterm3 = dual_term2_complex / dual_term3_complex;
		LoboComplex<COMPLEX_TYPE, double> root1_d, root2_d, root3_d;



		root1_d = dual_term3_complex / ((TYPE)3.0* CUBIC2) - CUBIC2 / (TYPE)3.0*(term2dterm3);
		root1 = trace_ / (TYPE)3.0 + root1_d.real_;

		/*root1 = trace_ / (TYPE)3.0 - CUBIC2*term2 / ((TYPE)3.0*term3)
			+ term3 / ((TYPE)3.0* CUBIC2);*/

		root2_d = sqrt3_*term2dterm3 / ((TYPE)3.0*CUBIC23)
			- nsqrt3_*dual_term3_complex / ((TYPE)6.0*CUBIC2);
		root2 = trace_ / (TYPE)3.0 + root2_d.real_;

		/*root2 = trace_ / (TYPE)3.0 + sqrt3*term2 / ((TYPE)3.0*CUBIC23*term3)
			- nsqrt3*term3 / ((TYPE)6.0*CUBIC2);*/

		root3_d = nsqrt3_*term2dterm3 / ((TYPE)3.0*CUBIC23)
			- sqrt3_*dual_term3_complex / ((TYPE)6.0*CUBIC2);
		root3 = trace_ / (TYPE)3.0 + root3_d.real_;
		/*root3 = trace_ / (TYPE)3.0 + nsqrt3*term2 / ((TYPE)3.0*CUBIC23*term3)
			- sqrt3*term3 / ((TYPE)6.0*CUBIC2);*/




		lobo::multi<COMPLEX_TYPE, TYPE>(data[1], data[8], tmp_value2);
		lobo::multi<COMPLEX_TYPE, TYPE>(data[2], data[7], tmp_value);
		tmp_value2 -= tmp_value;

		lobo::multi<COMPLEX_TYPE, TYPE>(data[1], data[5], tmp_value3);
		lobo::multi<COMPLEX_TYPE, TYPE>(data[2], data[4], tmp_value);
		tmp_value3 -= tmp_value;

		/*tmp_value2 = -data[2] * data[7] + data[1] * data[8];
		tmp_value3 = -data[2] * data[4] + data[1] * data[5];*/

		COMPLEX_TYPE tmp_term1 = (tmp_value2 - data[1] * root1);
		COMPLEX_TYPE tmp_term2 = (tmp_value3 + data[2] * root1);
		COMPLEX_TYPE tmp_term3;
		lobo::divide<COMPLEX_TYPE,TYPE>(tmp_term1, tmp_term2, tmp_value);

		//tmp_value = (tmp_value2 - data[1] * root1) / (tmp_value3 + data[2] * root1);
		result.data[0] = -(data[8] - root1) / data[2] + data[5] * tmp_value / data[2];
		result.data[3] = -tmp_value;
		result.data[6] = 1.0;

		/*result.data[0]
			= -(data[8] - root1) / data[2] + data[5] * (-data[2] * data[7] + data[1] * data[8] - data[1] * root1) / (data[2] * (-data[2] * data[4] + data[1] * data[5] + data[2] * root1));

		result.data[3]
			= -(-data[2] * data[7] + data[1] * data[8] - data[1] * root1) / (-data[2] * data[4] + data[1] * data[5] + data[2] * root1);
		result.data[6] = 1.0;*/
		tmp_term1 = (tmp_value2 - data[1] * root2);
		tmp_term2 = (tmp_value3 + data[2] * root2);
		lobo::divide<COMPLEX_TYPE, TYPE>(tmp_term1, tmp_term2, tmp_value);
		//tmp_value = (tmp_value2 - data[1] * root2) / (tmp_value3 + data[2] * root2);
		result.data[1] = -(data[8] - root2) / data[2] + data[5] * tmp_value / data[2];
		result.data[4] = -tmp_value;
		result.data[7] = 1.0;

		/*result.data[1]
			= -(data[8] - root2) / data[2] + data[5] * (-data[2] * data[7] + data[1] * data[8] - data[1] * root2) / (data[2] * (-data[2] * data[4] + data[1] * data[5] + data[2] * root2));
		result.data[4]
			= -(-data[2] * data[7] + data[1] * data[8] - data[1] * root2) / (-data[2] * data[4] + data[1] * data[5] + data[2] * root2);
		result.data[7] = 1.0;*/
		tmp_term1 = (tmp_value2 - data[1] * root3);
		tmp_term2 = (tmp_value3 + data[2] * root3);
		lobo::divide<COMPLEX_TYPE, TYPE>(tmp_term1, tmp_term2, tmp_value);
		//tmp_value = (tmp_value2 - data[1] * root3) / (tmp_value3 + data[2] * root3);
		result.data[2] = -(data[8] - root3) / data[2] + data[5] * tmp_value / data[2];
		result.data[5] = -tmp_value;
		result.data[8] = 1.0;

		/*result.data[2]
			= -(data[8] - root3) / data[2] + data[5] * (-data[2] * data[7] + data[1] * data[8] - data[1] * root3) / (data[2] * (-data[2] * data[4] + data[1] * data[5] + data[2] * root3));

		result.data[5]
			= -(-data[2] * data[7] + data[1] * data[8] - data[1] * root3) / (-data[2] * data[4] + data[1] * data[5] + data[2] * root3);
		result.data[8] = 1.0;*/

		

		std::vector<int> eigen_value_orders(3);
		eigen_value_orders[0] = 0;
		eigen_value_orders[1] = 1;
		eigen_value_orders[2] = 2;


		COMPLEX_TYPE tmpvalues[3];

		tmpvalues[0] = root1;
		tmpvalues[1] = root2;
		tmpvalues[2] = root3;

		int tmp;
		if (lobo::largerReal(tmpvalues[eigen_value_orders[0]], tmpvalues[eigen_value_orders[1]]))
		{
			tmp = eigen_value_orders[0];
			eigen_value_orders[0] = eigen_value_orders[1];
			eigen_value_orders[1] = tmp;
		}

		if (lobo::largerReal(tmpvalues[eigen_value_orders[1]], tmpvalues[eigen_value_orders[2]]))
		{
			tmp = eigen_value_orders[1];
			eigen_value_orders[1] = eigen_value_orders[2];
			eigen_value_orders[2] = tmp;
		}

		if (lobo::largerReal(tmpvalues[eigen_value_orders[0]], tmpvalues[eigen_value_orders[1]]))
		{
			tmp = eigen_value_orders[0];
			eigen_value_orders[0] = eigen_value_orders[1];
			eigen_value_orders[1] = tmp;
		}

		eigenvalues[0] = tmpvalues[eigen_value_orders[2]];
		eigenvalues[1] = tmpvalues[eigen_value_orders[1]];
		eigenvalues[2] = tmpvalues[eigen_value_orders[0]];


		LoboComplexMatrixTYPE3 ordered_result;

		//sort eigen value and eigen vectors
		//if(0)
		for (int i = 0;i < 3;i++)
		{
			COMPLEX_TYPE vectornorm;
			
			for (int j = 0;j < 3;j++)
			{
				lobo::multi<COMPLEX_TYPE, TYPE>(result.data[j * 3 + eigen_value_orders[i]], result.data[j * 3 + eigen_value_orders[i]], tmp_value2);
				vectornorm += tmp_value2;
			}
			lobo::powS<COMPLEX_TYPE,TYPE>(vectornorm, 0.5, tmp_value);
			//vectornorm = lobo::pow(vectornorm, (TYPE)0.5);

			for (int j = 0;j < 3;j++)
			{
				//result.data[j * 3 + eigen_value_orders[i]] /= tmp_value;
				ordered_result.data[j * 3 + 2 - i] = result.data[j * 3 + eigen_value_orders[i]] / tmp_value;
			}
		}

		result = ordered_result;
	}

}