#pragma once

#include "TypeIsotropicMaterial.h"

template <class TYPE>
class TypeMaterialWithCompressionResistance:public TypeIsotropicMaterial<TYPE>
{
	typedef Eigen::Matrix<TYPE, 3, 3> Matrix3;
	typedef Eigen::Matrix<TYPE, -1, -1> MatrixX;

public:
	TypeMaterialWithCompressionResistance(bool enableCompressionResistance = 0);
	~TypeMaterialWithCompressionResistance();


	virtual TYPE ComputeEnergy(int elementIndex, TYPE * invariants) = 0;
	virtual void ComputeEnergyGradient(int elementIndex, TYPE * invariants, TYPE * gradient) = 0; // invariants and gradient are 3-vectors
	virtual void ComputeEnergyHessian(int elementIndex, TYPE * invariants, TYPE * hessian) = 0; // invariants is a 3-vector, hessian is a 3x3 symmetric matrix, unrolled into a 6-vector, in the following order: (11, 12, 13, 22, 23, 33).
	virtual void computeImaginaryP(int elementIndex, Matrix3 &F, Matrix3 &iF, Matrix3 &P, TYPE h) = 0;

	virtual void computeImaginaryEnergyVector(int elementIndex, TYPE* forces, Matrix3 &F, TYPE h) = 0;
	virtual void computeAutoDiffEnergyVector(int elementIndex, TYPE* forces, Matrix3& F, TYPE& energy) = 0;

	virtual void computeAutoDiffEnergy(int elementIndex, Matrix3& F, TYPE& energy, TYPE scale) = 0;

	virtual void computeAutoDiffEnergyVectorDiagMatrix(int elementIndex, TYPE* forces, MatrixX *stiffness, Matrix3& F, TYPE& energy) = 0;
	virtual void computeAutoDiffEnergyVectorMatrix(int elementIndex, TYPE* forces, MatrixX *stiffness, Matrix3& F, TYPE& energy) = 0;

	virtual void computeDirectionalHessianMatrix(int elementIndex, TYPE* ele_u, MatrixX *stiffness, TYPE& energy, TYPE h) = 0;

protected:

	bool enableCompressionResistance;

	virtual TYPE GetCompressionResistanceFactor(int elementIndex);

	void AddCompressionResistanceEnergy(int elementIndex, TYPE * invariants, TYPE * energy);

	void AddCompressionResistanceGradient(int elementIndex, TYPE * invariants, TYPE * gradient);
	void AddCompressionResistanceHessian(int elementIndex, TYPE * invariants, TYPE * hessian);




};

