#include "TypeNeoHookeanMaterial.h"
#include <complex>
#include <omp.h>
#include "Functions/LoboMacros.h"

LOBO_TEMPLATE_INSTANT(TypeNeoHookeanMaterial)

template <class TYPE>
TypeNeoHookeanMaterial<TYPE>::TypeNeoHookeanMaterial(Lobo::LoboTetMesh* tetmesh, int enableCompressionResistance /*= 0*/, TYPE compressionResistance /*= 0.0*/):TypeTetElementMaterial<TYPE>(tetmesh, enableCompressionResistance, compressionResistance)
{

}

template <class TYPE>
TypeNeoHookeanMaterial<TYPE>::~TypeNeoHookeanMaterial()
{
	
}


#define ENERGY_FUNCTION(LoboComplex_NAME)\
E_complex = F_complex.transpose()*F_complex;\
LoboComplex_NAME I1 = E_complex.trace();\
LoboComplex_NAME J = F_complex.det();\
LoboComplex_NAME A = lobo::pow(J, (TYPE)(-2.0 / 3.0));\
LoboComplex_NAME I1_ = A*I1;\
LoboComplex_NAME J_1_2 = J - (TYPE)1.0;\
LoboComplex_NAME e_j = J_1_2*J_1_2;\
energy = muLame[elementIndex] * (I1_ - (TYPE)3.0) + lambdaLame[elementIndex] * e_j;

//#define ENERGY_FUNCTION(LoboComplex_NAME)\
//E_complex = F_complex.transpose()*F_complex;\
//LoboComplex_NAME det_F = F_complex.det();\
//LoboComplex_NAME log_det_F = lobo::log(det_F);\
//E_complex = F_complex.transpose()*F_complex;\
//LoboComplex_NAME I1 = E_complex.trace();\
//energy = muLame[elementIndex] * (TYPE)0.5*(I1 - (TYPE)3.0);\
//energy -= muLame[elementIndex] * log_det_F;\
//energy += lambdaLame[elementIndex] * (TYPE)0.5 * log_det_F*log_det_F;



template <class TYPE>
TYPE TypeNeoHookeanMaterial<TYPE>::ComputeEnergy(int elementIndex, TYPE * invariants)
{
	TYPE IC = invariants[0];
	TYPE IIIC = invariants[2];
	TYPE J = std::sqrt(IIIC);
	TYPE logJ = std::log(J);


	TYPE energy = 0.5 * muLame[elementIndex] * (IC - 3.0) - muLame[elementIndex] * logJ + 0.5 * lambdaLame[elementIndex] * logJ * logJ;

	this->AddCompressionResistanceEnergy(elementIndex, invariants, &energy);
	return energy;

}

template <class TYPE>
void TypeNeoHookeanMaterial<TYPE>::ComputeEnergyGradient(int elementIndex, TYPE * invariants, TYPE * gradient)
{
	TYPE IIIC = invariants[2];
	gradient[0] = 0.5 * muLame[elementIndex];
	gradient[1] = 0.0;
	gradient[2] = (-0.5 * muLame[elementIndex] + 0.25 * lambdaLame[elementIndex] * std::log(IIIC)) / IIIC;

	this->AddCompressionResistanceGradient(elementIndex, invariants, gradient);
}

template <class TYPE>
void TypeNeoHookeanMaterial<TYPE>::ComputeEnergyHessian(int elementIndex, TYPE * invariants, TYPE * hessian)
{
	TYPE IIIC = invariants[2];
	// 11
	hessian[0] = 0.0;
	// 12
	hessian[1] = 0.0;
	// 13
	hessian[2] = 0.0;
	// 22
	hessian[3] = 0.0;
	// 23
	hessian[4] = 0.0;
	// 33
	hessian[5] = (0.25 * lambdaLame[elementIndex] + 0.5 * muLame[elementIndex] - 0.25 * lambdaLame[elementIndex] * std::log(IIIC)) / (IIIC * IIIC);

	this->AddCompressionResistanceHessian(elementIndex, invariants, hessian);
}



template <class TYPE>
void TypeNeoHookeanMaterial<TYPE>::computeImaginaryP(int elementIndex, Matrix3 &F, Matrix3 &iF, Matrix3 &P, TYPE h)
{
	std::vector<std::complex<TYPE>> F_complex(9);
	for (int i = 0;i < 9;i++)
	{
		F_complex[i] = { F.data()[i],iF.data()[i] };
	}
	
	//compute det
	std::complex<TYPE> det_F(0,0);

	det_F = F_complex[0] * (F_complex[4] * F_complex[8] - F_complex[5] * F_complex[7])
		- F_complex[3] * (F_complex[1] * F_complex[8] - F_complex[2] * F_complex[7])
		+ F_complex[6] * (F_complex[1] * F_complex[5] - F_complex[2] * F_complex[4]);

	//compute inverse
	std::vector<std::complex<TYPE>> F_complex_inverse(9);
	
	std::complex<TYPE> det_F_inv = std::complex<TYPE>(1.0,0.0) / det_F;

	F_complex_inverse[0] = (F_complex[4] * F_complex[8] - F_complex[5] * F_complex[7])*det_F_inv;
	F_complex_inverse[3] = (F_complex[6] * F_complex[5] - F_complex[3] * F_complex[8])*det_F_inv;
	F_complex_inverse[6] = (F_complex[3] * F_complex[7] - F_complex[6] * F_complex[4])*det_F_inv;

	F_complex_inverse[1] = (F_complex[7] * F_complex[2] - F_complex[1] * F_complex[8])*det_F_inv;
	F_complex_inverse[4] = (F_complex[0] * F_complex[8] - F_complex[6] * F_complex[2])*det_F_inv;
	F_complex_inverse[7] = (F_complex[1] * F_complex[6] - F_complex[0] * F_complex[7])*det_F_inv;

	F_complex_inverse[2] = (F_complex[1] * F_complex[5] - F_complex[2] * F_complex[4])*det_F_inv;
	F_complex_inverse[5] = (F_complex[2] * F_complex[3] - F_complex[0] * F_complex[5])*det_F_inv;
	F_complex_inverse[8] = (F_complex[0] * F_complex[4] - F_complex[1] * F_complex[3])*det_F_inv;

	std::complex<TYPE> log_J = std::log(det_F);

	//compute log J F_inverse
	std::vector<TYPE> logj_F_complex_inverse(9);

	logj_F_complex_inverse[0] =lambdaLame[elementIndex]*( log_J.real()*F_complex_inverse[0].imag() + log_J.imag()*F_complex_inverse[0].real());
	logj_F_complex_inverse[1] = lambdaLame[elementIndex] * (log_J.real()*F_complex_inverse[3].imag() + log_J.imag()*F_complex_inverse[3].real());
	logj_F_complex_inverse[2] = lambdaLame[elementIndex] * (log_J.real()*F_complex_inverse[6].imag() + log_J.imag()*F_complex_inverse[6].real());

	logj_F_complex_inverse[3] = lambdaLame[elementIndex] * (log_J.real()*F_complex_inverse[1].imag() + log_J.imag()*F_complex_inverse[1].real());
	logj_F_complex_inverse[4] = lambdaLame[elementIndex] * (log_J.real()*F_complex_inverse[4].imag() + log_J.imag()*F_complex_inverse[4].real());
	logj_F_complex_inverse[5] = lambdaLame[elementIndex] * (log_J.real()*F_complex_inverse[7].imag() + log_J.imag()*F_complex_inverse[7].real());

	logj_F_complex_inverse[6] = lambdaLame[elementIndex] * (log_J.real()*F_complex_inverse[2].imag() + log_J.imag()*F_complex_inverse[2].real());
	logj_F_complex_inverse[7] = lambdaLame[elementIndex] * (log_J.real()*F_complex_inverse[5].imag() + log_J.imag()*F_complex_inverse[5].real());
	logj_F_complex_inverse[8] = lambdaLame[elementIndex] * (log_J.real()*F_complex_inverse[8].imag() + log_J.imag()*F_complex_inverse[8].real());


	P.data()[0] =muLame[elementIndex]* (F_complex[0].imag() - F_complex_inverse[0].imag()) + logj_F_complex_inverse[0];
	P.data()[1] = muLame[elementIndex] * (F_complex[1].imag() - F_complex_inverse[3].imag()) + logj_F_complex_inverse[1];
	P.data()[2] = muLame[elementIndex] * (F_complex[2].imag() - F_complex_inverse[6].imag()) + logj_F_complex_inverse[2];

	P.data()[3] = muLame[elementIndex] * (F_complex[3].imag() - F_complex_inverse[1].imag()) + logj_F_complex_inverse[3];
	P.data()[4] = muLame[elementIndex] * (F_complex[4].imag() - F_complex_inverse[4].imag()) + logj_F_complex_inverse[4];
	P.data()[5] = muLame[elementIndex] * (F_complex[5].imag() - F_complex_inverse[7].imag()) + logj_F_complex_inverse[5];

	P.data()[6] = muLame[elementIndex] * (F_complex[6].imag() - F_complex_inverse[2].imag()) + logj_F_complex_inverse[6];
	P.data()[7] = muLame[elementIndex] * (F_complex[7].imag() - F_complex_inverse[5].imag()) + logj_F_complex_inverse[7];
	P.data()[8] = muLame[elementIndex] * (F_complex[8].imag() - F_complex_inverse[8].imag()) + logj_F_complex_inverse[8];
	//system("pause");

}

template <class TYPE>
void TypeNeoHookeanMaterial<TYPE>::computeImaginaryEnergyVector(int elementIndex, TYPE* forces, Matrix3 &F, TYPE h)
{

}


template <class TYPE>
void TypeNeoHookeanMaterial<TYPE>::computeAutoDiffEnergy(int elementIndex, Matrix3& F_complex, TYPE& energy, TYPE scale)
{
	Matrix3 E_complex;

	E_complex = F_complex.transpose()*F_complex;
	TYPE I1 = E_complex.trace();
	TYPE J = F_complex.determinant();
	TYPE A = std::pow(J, (TYPE)(-2.0 / 3.0));
	TYPE I1_ = A*I1;
	TYPE J_1_2 = J - (TYPE)1.0;
	TYPE e_j = J_1_2*J_1_2;
	energy = scale*muLame[elementIndex] * (I1_ - (TYPE)3.0) + scale*lambdaLame[elementIndex] * e_j;

}

template <class TYPE>
void TypeNeoHookeanMaterial<TYPE>::computeAutoDiffEnergyVector(int elementIndex, TYPE* forces, Matrix3& F, TYPE& energy_)
{
	lobo::LoboComplexMatrix3<LoboComplext, TYPE> F_complex, E_complex;
	LoboComplext energy;
	for (int r = 0; r < 12;r++)
	{
		F_complex = element_dF_du[elementIndex * 12 + r];
		F_complex.setRealValue(F.data());


		ENERGY_FUNCTION(LoboComplext)

			/*E_complex = F_complex.transpose()*F_complex;
			LoboComplext I1 = E_complex.trace();
			LoboComplext J = F_complex.det();
			LoboComplext A = lobo::pow(J, (TYPE)(-2.0 / 3.0));
			LoboComplext I1_ = A*I1;
			LoboComplext J_1_2 = J - (TYPE)1.0;
			LoboComplext e_j = J_1_2*J_1_2;
			energy = muLame[elementIndex] * (I1_ - (TYPE)3.0) + lambdaLame[elementIndex] * e_j;*/


		/*LoboComplext det_F = F_complex.det();
		LoboComplext log_det_F = lobo::log(det_F);
		E_complex = F_complex.transpose()*F_complex;
		LoboComplext I1 = E_complex.trace();
		energy = muLame[elementIndex] * (TYPE)0.5*(I1 - (TYPE)3.0);
		energy -= muLame[elementIndex] * log_det_F;
		energy += lambdaLame[elementIndex] * (TYPE)0.5 * log_det_F*log_det_F;*/
		forces[r] = energy.image_/this->h_CSFD;
		energy_ = energy.real_;
	}
}

template <class TYPE>
void TypeNeoHookeanMaterial<TYPE>::computeAutoDiffEnergyVectorDiagMatrix(int elementIndex, TYPE* forces, MatrixX *stiffness, Matrix3& F, TYPE& energy_)
{
	stiffness->setZero();
	lobo::LoboComplexMatrix3<LoboComplexDualt, TYPE> F_complex, E_complex;
	LoboComplexDualt energy;
	for (int r = 0; r < 12;r++)
	{
		F_complex = element_dF_dudu[elementIndex * 12 + r];
		F_complex.setRealValue(F.data());

		ENERGY_FUNCTION(LoboComplexDualt)


			/*E_complex = F_complex.transpose()*F_complex;
			LoboComplexDualt I1 = E_complex.trace();
			LoboComplexDualt J = F_complex.det();
			LoboComplexDualt A = lobo::pow(J, (TYPE)(-2.0 / 3.0));
			LoboComplexDualt I1_ = A*I1;
			LoboComplexDualt J_1_2 = J - (TYPE)1.0;
			LoboComplexDualt e_j = J_1_2*J_1_2;
			energy = muLame[elementIndex] * (I1_ - (TYPE)3.0) + lambdaLame[elementIndex] * e_j;*/


		/*LoboComplexDualt det_F = F_complex.det();
		LoboComplexDualt log_det_F = lobo::log(det_F);
		E_complex = F_complex.transpose()*F_complex;
		LoboComplexDualt I1 = E_complex.trace();
		energy = muLame[elementIndex] * (TYPE)0.5*(I1 - (TYPE)3.0);
		energy -= muLame[elementIndex] * log_det_F;
		energy += lambdaLame[elementIndex] * (TYPE)0.5 * log_det_F*log_det_F;*/
		forces[r] = energy.real_.image_;
		stiffness->data()[r * 12 + r] = energy.image_.image_;
		energy_ = energy.real_.real_;
		/*forces[r] = energy.image_;
		energy_ = energy.real_;*/
	}
}

template <class TYPE>
void TypeNeoHookeanMaterial<TYPE>::computeAutoDiffEnergyVectorMatrix(int elementIndex, TYPE* forces, MatrixX *stiffness, Matrix3& F, TYPE& energy_)
{
	stiffness->setZero();
	lobo::LoboComplexMatrix3<LoboComplexDualt, TYPE> F_complex, E_complex;
	LoboComplexDualt energy;

	int index = 0;
	for (int u = 0;u < 12;u++)
	{
		for (int v = 0;v < u + 1;v++)
		{
			//std::cout << elementIndex * 78 + index <<" "<< u <<" " << v << std::endl;
			//std::cout << element_dF_dudv.size() << std::endl;

			F_complex = element_dF_dudv[elementIndex * 78 + index];
			F_complex.setRealValue(F.data());


			ENERGY_FUNCTION(LoboComplexDualt);
			
				/*E_complex = F_complex.transpose()*F_complex;
				LoboComplexDualt I1 = E_complex.trace();
				LoboComplexDualt J = F_complex.det();
				LoboComplexDualt A = lobo::pow(J, (TYPE)(-2.0 / 3.0));
				LoboComplexDualt I1_ = A*I1;
				LoboComplexDualt J_1_2 = J - (TYPE)1.0;
				LoboComplexDualt e_j = J_1_2*J_1_2;
				energy = muLame[elementIndex] * (I1_ - (TYPE)3.0)+ lambdaLame[elementIndex] * e_j;*/


			/*LoboComplexDualt det_F = F_complex.det();
			LoboComplexDualt log_det_F = lobo::log(det_F);
			E_complex = F_complex.transpose()*F_complex;
			LoboComplexDualt I1 = E_complex.trace();
			energy = muLame[elementIndex] * (TYPE)0.5*(I1 - (TYPE)3.0);
			energy -= muLame[elementIndex] * log_det_F;
			energy += lambdaLame[elementIndex] * (TYPE)0.5 * log_det_F*log_det_F;*/

			if (u == v)
			{
				forces[u] = energy.real_.image_/this->h_CSFD;
			}

			stiffness->data()[v * 12 + u] = energy.image_.image_/this->h_CSFD/this->h_CSFD;
			energy_ = energy.real_.real_;

			index++;
		}
	}

	//fill the matrix
	for (int u = 0;u < 12;u++)
	{
		for (int v = u+1;v < 12;v++)
		{
			stiffness->data()[v * 12 + u] = stiffness->data()[u * 12 + v];
		}
	}
	
}

template <class TYPE>
void TypeNeoHookeanMaterial<TYPE>::computeDirectionalHessianMatrix(int elementIndex, TYPE* ele_u, MatrixX *stiffness, TYPE& energy_,TYPE h)
{
	stiffness->setZero();
	lobo::LoboComplexMatrix3<LoboComplexTrit, TYPE> Ds_dualcomplex,F_complex, E_complex;
	LoboComplexTrit energy;
	Lobo::TetElementData *te = tetmesh->getTetElement(elementIndex);
	Matrix3 Dm_inverse = te->Dm_inverse.cast<TYPE>();

	int index = 0;
	for (int u = 0;u < 12;u++)
	{
		for (int v = 0;v < u + 1;v++)
		{
			//F_complex_list[elementIndex].setRealValue(F.data());

			LoboComplexTrit xjj[12];
			for (int i = 0;i < 4;i++)
			{
				for (int j = 0;j < 3;j++)
				{
					xjj[i * 3 + j].setZero();
				}
			}

			xjj[u].real_.image_.real_ = h;
			xjj[v].image_.real_.real_ = h;
			for (int i = 0;i < 12;i++)
			{
				xjj[i].real_.real_.image_ = ele_u[i] * h;
			}

			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					Ds_dualcomplex.data[j * 3 + i] = xjj[j * 3 + i] - xjj[9 + i];
				}
			}

			Matrix3 Dm = te->Dm.cast<TYPE>();
			

			Ds_dualcomplex = Ds_dualcomplex + Dm.data();
			F_complex = Ds_dualcomplex*Dm_inverse.data();
			F_complex.data[0] = (TYPE)1.0;
			F_complex.data[4] = (TYPE)1.0;
			F_complex.data[8] = (TYPE)1.0;
			

			ENERGY_FUNCTION(LoboComplexTrit);

			stiffness->data()[v * 12 + u] = energy.image_.image_.image_;
			energy_ = energy.real_.real_.real_;

			index++;
		}
	}

	for (int u = 0;u < 12;u++)
	{
		for (int v = u + 1;v < 12;v++)
		{
			stiffness->data()[v * 12 + u] = stiffness->data()[u * 12 + v];
		}
	}
}



