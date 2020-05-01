#pragma once
#include "TypeMaterialWithCompressionResistance.h"
#include "LoboVolumtricMesh/LoboTetMesh.h"
#include "AutoDiff/AutoDiffCore.h"
#include <vector>
//adapter between TypeMaterialWithCompressionResistance and material based on the tet mesh

template <class TYPE>
class TypeTetElementMaterial: public TypeMaterialWithCompressionResistance<TYPE>
{
	typedef Eigen::Matrix<TYPE, 3, 3> Matrix3;
	typedef Eigen::Matrix<TYPE, -1, -1> MatrixX;

public:

	LOBO_MAKE_TYPEDEFS(TYPE, t);


	TypeTetElementMaterial(Lobo::LoboTetMesh* tetmesh, int enableCompressionResistance = 0, TYPE compressionResistance = 0.0);
	~TypeTetElementMaterial();

	virtual void updateMaterial();

	virtual TYPE ComputeEnergy(int elementIndex, TYPE * invariants) = 0;
	virtual void ComputeEnergyGradient(int elementIndex, TYPE * invariants, TYPE * gradient) = 0; // invariants and gradient are 3-vectors
	virtual void ComputeEnergyHessian(int elementIndex, TYPE * invariants, TYPE * hessian) = 0; // invariants is a 3-vector, hessian is a 3x3 symmetric matrix, unrolled into a 6-vector, in the following order: (11, 12, 13, 22, 23, 33).
	virtual void computeImaginaryP(int elementIndex, Matrix3 &F, Matrix3 &iF, Matrix3 &P, TYPE h) = 0;

	virtual void computeImaginaryEnergyVector(int elementIndex, TYPE* forces, Matrix3 &F, TYPE h) = 0;
	virtual void computeAutoDiffEnergyVector(int elementIndex, TYPE* forces, Matrix3& F, TYPE& energy) = 0;
	virtual void computeAutoDiffEnergy(int elementIndex, Matrix3& F, TYPE& energy, TYPE scale) {};



	virtual void computeAutoDiffEnergyVectorDiagMatrix(int elementIndex, TYPE* forces, MatrixX *stiffness, Matrix3& F, TYPE& energy) = 0;

	virtual void computeDirectionalHessianMatrix(int elementIndex, TYPE* ele_u, MatrixX *stiffness, TYPE& energy, TYPE h) {};

	virtual void getFComplex(int elementIndex, Matrix3&F,lobo::LoboComplexMatrix3<LoboComplexDualt, TYPE> &F_complex_output,int u,int v);
	virtual void getFComplex_single(int elementIndex, Matrix3&F, lobo::LoboComplexMatrix3<LoboComplext, TYPE> &F_complex_output, int u);

	virtual void getLambdaLame(int eleid, TYPE &lambda);
	virtual void getMuLame(int eleid, TYPE &mu);

protected:

	TYPE* lambdaLame;
	TYPE* muLame;

	TYPE compressionResistance;
	TYPE * EdivNuFactor;
	virtual TYPE GetCompressionResistanceFactor(int elementIndex);

	Lobo::LoboTetMesh* tetmesh;

	std::vector<lobo::LoboComplexMatrix3<LoboComplext, TYPE>> element_dF_du;
	std::vector<lobo::LoboComplexMatrix3<LoboComplexDualt, TYPE>> element_dF_dudu;
	std::vector<lobo::LoboComplexMatrix3<LoboComplexDualt, TYPE>> element_dF_dudv;


	//some usefull function to compute dJ_du or something
	virtual void derivativeFdetThreeTerm(std::vector<int>& term_index, const lobo::LoboComplexMatrix3<LoboComplext, TYPE> &F,TYPE& real_part,TYPE&image_part, bool imageonly = false);

	virtual void secondDerivativeFdetThreeTerm(std::vector<int>& term_index, lobo::LoboComplexMatrix3<LoboComplexDualt, TYPE> &F, TYPE& real_part, TYPE&image_part,bool imageonly = false);

	virtual void secondDerivativeFdet(lobo::LoboComplexMatrix3<LoboComplexDualt, TYPE> &F, TYPE& real_part, TYPE&image_part, bool imageonly = false);

	virtual void derivativeFdet(int r, const lobo::LoboComplexMatrix3<LoboComplext, TYPE> &F, TYPE& real_part, TYPE&image_part,bool imageonly=false);


	virtual void derivativeI1(lobo::LoboComplexMatrix3<LoboComplext, TYPE> &F, TYPE& real_part, TYPE&image_part,bool imageonly =false);
	virtual void secondDerviativeI1(lobo::LoboComplexMatrix3<LoboComplexDualt, TYPE> &F, TYPE& real_part, TYPE&image_part);


	virtual void derivativeE(lobo::LoboComplexMatrix3<LoboComplext, TYPE> &F, lobo::LoboComplexMatrix3<LoboComplext, TYPE> &E);

	virtual void derivativeI2(lobo::LoboComplexMatrix3<LoboComplext, TYPE> &F, TYPE& real_part, TYPE&image_part,bool real_part_need = false);

	virtual void secondDerivativeI2(lobo::LoboComplexMatrix3<LoboComplexDualt, TYPE> &F, TYPE& real_part, TYPE&image_part, bool real_part_need = false);

	//lobo::LoboComplexMatrix3<LoboComplext, TYPE> F_complex, E_complex;
	std::vector<lobo::LoboComplexMatrix3<LoboComplext, TYPE>> F_complex_list;
	std::vector<lobo::LoboComplexMatrix3<LoboComplexDualt, TYPE>> F_dualcomplex_list;

	//std::vector<lobo::LoboComplexMatrix3<LoboComplext, TYPE>> E_complex_list;

};

template <class TYPE>
void TypeTetElementMaterial<TYPE>::getMuLame(int eleid, TYPE &mu)
{
	mu = muLame[eleid];
}

template <class TYPE>
void TypeTetElementMaterial<TYPE>::getLambdaLame(int eleid, TYPE &lambda)
{
	lambda = lambdaLame[eleid];
}

