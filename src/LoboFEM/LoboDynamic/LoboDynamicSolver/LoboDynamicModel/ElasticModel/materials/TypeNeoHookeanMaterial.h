#pragma once
#include "TypeTetElementMaterial.h"

template <class TYPE>
class TypeNeoHookeanMaterial:public TypeTetElementMaterial<TYPE>
{
	typedef Eigen::Matrix<TYPE, 3, 3> Matrix3;
	typedef Eigen::Matrix<TYPE, -1, -1> MatrixX;

	LOBO_MAKE_TYPEDEFS(TYPE, t);

public:
using TypeTetElementMaterial<TYPE>::lambdaLame;
	using TypeTetElementMaterial<TYPE>::muLame;
	using TypeTetElementMaterial<TYPE>::compressionResistance;
	using TypeTetElementMaterial<TYPE>::EdivNuFactor;
	using TypeTetElementMaterial<TYPE>::tetmesh;
	using TypeTetElementMaterial<TYPE>::element_dF_du;
	using TypeTetElementMaterial<TYPE>::element_dF_dudu;
	using TypeTetElementMaterial<TYPE>::element_dF_dudv;
	TypeNeoHookeanMaterial(Lobo::LoboTetMesh* tetmesh, int enableCompressionResistance = 0, TYPE compressionResistance = 0.0);
	~TypeNeoHookeanMaterial();

	virtual TYPE ComputeEnergy(int elementIndex, TYPE * invariants);
	virtual void ComputeEnergyGradient(int elementIndex, TYPE * invariants, TYPE * gradient); // invariants and gradient are 3-vectors
	virtual void ComputeEnergyHessian(int elementIndex, TYPE * invariants, TYPE * hessian); // invariants is a 3-vector, hessian is a 3x3 symmetric matrix, unrolled into a 6-vector, in the following order: (11, 12, 13, 22, 23, 33).
	virtual void computeImaginaryP(int elementIndex, Matrix3 &F, Matrix3 &iF, Matrix3 &P, TYPE h);
	virtual void computeImaginaryEnergyVector(int elementIndex, TYPE* forces, Matrix3 &F, TYPE h);
	virtual void computeAutoDiffEnergyVector(int elementIndex, TYPE* forces, Matrix3& F, TYPE& energy);
	virtual void computeAutoDiffEnergy(int elementIndex, Matrix3& F, TYPE& energy, TYPE scale);
	virtual void computeAutoDiffEnergyVectorDiagMatrix(int elementIndex, TYPE* forces, MatrixX *stiffness, Matrix3& F, TYPE& energy);
	virtual void computeAutoDiffEnergyVectorMatrix(int elementIndex, TYPE* forces, MatrixX *stiffness, Matrix3& F, TYPE& energy);


	virtual void computeDirectionalHessianMatrix(int elementIndex, TYPE* ele_u, MatrixX *stiffness, TYPE& energy_, TYPE h);

protected:
	

};

