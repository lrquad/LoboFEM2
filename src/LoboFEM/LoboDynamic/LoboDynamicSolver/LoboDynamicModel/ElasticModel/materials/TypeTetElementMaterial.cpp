#include "TypeTetElementMaterial.h"


LOBO_TEMPLATE_INSTANT(TypeTetElementMaterial)

template <class TYPE>
TypeTetElementMaterial<TYPE>::TypeTetElementMaterial(Lobo::LoboTetMesh* tetmesh, int enableCompressionResistance /*= 0*/, TYPE compressionResistance /*= 0.0*/)
{
	this->enableCompressionResistance = enableCompressionResistance;
	this->tetmesh = tetmesh;
	int numElements = tetmesh->getNumElements();
	lambdaLame = (TYPE*)malloc(sizeof(TYPE) * numElements);
	muLame = (TYPE*)malloc(sizeof(TYPE) * numElements);

	if (enableCompressionResistance)
		EdivNuFactor = (TYPE*)malloc(sizeof(TYPE) * numElements);
	else
		EdivNuFactor = NULL;

	#pragma omp parallel for
	for (int el = 0; el < numElements; el++)
	{
		Lobo::Material* material = tetmesh->getElementMaterial(el);

		if (material == NULL)
		{
			printf("Error: NeoHookeanIsotropicMaterial: mesh does not consist of E, nu materials.\n");
			throw 1;
		}

		lambdaLame[el] = (TYPE)material->getLambda();
		muLame[el] = (TYPE)material->getMu();

		if (enableCompressionResistance)
		{
			EdivNuFactor[el] = compressionResistance * (TYPE)material->getE() / (1.0 - 2.0 * (TYPE)material->getNu());
			//printf("Setting EdivNuFactor[%d]=%G\n", el, EdivNuFactor[el]);
		}
	}
	
	TYPE h = this->h_CSFD;
	element_dF_du.resize(numElements * 12);
	element_dF_dudu.resize(numElements * 12);
	element_dF_dudv.resize(numElements * 78);

	lobo::LoboComplexMatrix3<LoboComplext, TYPE> Ds_complex;
	lobo::LoboComplexMatrix3<LoboComplexDualt, TYPE> Ds_dualcomplex;
	
	F_complex_list.resize(numElements);
	F_dualcomplex_list.resize(numElements);

	for (int i = 0; i < numElements; i++)
	{
		//F_complex_list.push_back(lobo::LoboComplexMatrix3<LoboComplext, TYPE>());
		Lobo::TetElementData* te =  tetmesh->getTetElement(i);
		Matrix3 Dm_inverse = te->Dm_inverse.cast<TYPE>();

		int index = 0;
		for (int u = 0;u < 12;u++)
		{
			for (int v = 0;v < u+1;v++)
			{
				LoboComplexDualt xjj[12];
				for (int i = 0;i < 4;i++)
				{
					for (int j = 0;j < 3;j++)
					{
						xjj[i * 3 + j].setZero();
					}
				}

				xjj[u].real_.image_ = h;
				xjj[v].image_.real_ = h;

				for (int i = 0; i < 3; i++)
				{
					for (int j = 0; j < 3; j++)
					{
						Ds_dualcomplex.data[j * 3 + i] = xjj[j * 3 + i] - xjj[9 + i];
					}
				}

				Matrix3 Dm = te->Dm.cast<TYPE>();
				Ds_dualcomplex = Ds_dualcomplex + Dm.data();
				element_dF_dudv[i * 78 + index] = Ds_dualcomplex*Dm_inverse.data();

				

				index++;
			}
		}

		for (int r = 0;r < 12;r++)
		{
			LoboComplext xj[12];
			LoboComplexDualt xjj[12];

			for (int i = 0;i < 4;i++)
			{
				for (int j = 0;j < 3;j++)
				{
					xj[i * 3 + j].setZero();
					xjj[i * 3 + j].setZero();
				}
			}

			xj[r].image_ = h;

			xjj[r].real_.image_ = h;
			xjj[r].image_.real_ = h;

			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					Ds_complex.data[j * 3 + i] = xj[j * 3 + i] - xj[9 + i];
					Ds_dualcomplex.data[j * 3 + i] = xjj[j * 3 + i] - xjj[9 + i];
				}
			}

			Matrix3 Dm = te->Dm.cast<TYPE>();
			Ds_complex = Ds_complex + Dm.data();
			Ds_dualcomplex = Ds_dualcomplex + Dm.data();

			element_dF_du[i * 12 + r] = Ds_complex*Dm_inverse.data();
			element_dF_dudu[i * 12 + r] = Ds_dualcomplex*Dm_inverse.data();
		}
	}
}

template <class TYPE>
TypeTetElementMaterial<TYPE>::~TypeTetElementMaterial()
{
	free(EdivNuFactor);
	free(lambdaLame);
	free(muLame);
}

template <class TYPE>
void TypeTetElementMaterial<TYPE>::updateMaterial()
{
	std::cout << "updateMaterial  " << std::endl;
	int numElements = tetmesh->getNumElements();

	for (int el = 0; el < numElements; el++)
	{
		Lobo::Material* material = tetmesh->getElementMaterial(el);
		//LoboVolumetricMesh::Material * material = tetmesh->getElementMaterial(el);
		//LoboVolumetricMesh::ENuMaterial * eNuMaterial = downcastENuMaterial(material);
		if (material == NULL)
		{
			printf("Error: NeoHookeanIsotropicMaterial: mesh does not consist of E, nu materials.\n");
			throw 1;
		}

		lambdaLame[el] = (TYPE)material->getLambda();
		muLame[el] = (TYPE)material->getMu();

		if (this->enableCompressionResistance)
		{
			EdivNuFactor[el] = compressionResistance * (TYPE)material->getE() / (1.0 - 2.0 * (TYPE)material->getNu());
			//printf("Setting EdivNuFactor[%d]=%G\n", el, EdivNuFactor[el]);
		}
	}


	

}

template <class TYPE>
TYPE TypeTetElementMaterial<TYPE>::GetCompressionResistanceFactor(int elementIndex)
{
	return EdivNuFactor[elementIndex];
}

#define d .image_.real_
#define b .real_.image_
#define re .real_.real_
#define uv .image_.image_


template <class TYPE>
void TypeTetElementMaterial<TYPE>::secondDerivativeFdetThreeTerm(std::vector<int>& term_index, lobo::LoboComplexMatrix3<LoboComplexDualt, TYPE> &F, TYPE& real_part, TYPE&image_part, bool imageonly)
{
	real_part = 0;
	image_part = 0;

	if (!imageonly)
	{
		real_part = F.data[term_index[0]].real_.real_ * F.data[term_index[1]].real_.real_ *F.data[term_index[2]].real_.real_;
	}

	image_part = F.data[term_index[0]]d*F.data[term_index[1]]b*F.data[term_index[2]].real_.real_;
	image_part += F.data[term_index[0]]b*F.data[term_index[1]]d*F.data[term_index[2]].real_.real_;
	
	image_part += F.data[term_index[0]]d*F.data[term_index[1]].real_.real_*F.data[term_index[2]]b;
	image_part += F.data[term_index[0]]b*F.data[term_index[1]].real_.real_*F.data[term_index[2]]d;

	image_part += F.data[term_index[0]].real_.real_*F.data[term_index[1]]b*F.data[term_index[2]]d;
	image_part += F.data[term_index[0]].real_.real_*F.data[term_index[1]]d*F.data[term_index[2]]b;


	//TYPE tmp = 2.0 * dU*dV + 2.0 * real_part*image_part;
	//image_part = tmp;

	//real_part *= real_part;
}

template <class TYPE>
void TypeTetElementMaterial<TYPE>::derivativeFdetThreeTerm(std::vector<int>& term_index, const lobo::LoboComplexMatrix3<LoboComplext, TYPE> &F, TYPE& real_part, TYPE&image_part,bool imageonly)
{
	real_part = 0;
	image_part = 0;

	if (!imageonly)
	{
		real_part = F.data[term_index[0]].real_ * F.data[term_index[1]].real_ * F.data[term_index[2]].real_;
	}

	image_part = F.data[term_index[0]].image_*F.data[term_index[1]].real_*F.data[term_index[2]].real_ +
		F.data[term_index[0]].real_*F.data[term_index[1]].image_*F.data[term_index[2]].real_ +
		F.data[term_index[0]].real_*F.data[term_index[1]].real_*F.data[term_index[2]].image_;

	/*image_part -= F.data[term_index[0]].image_ * F.data[term_index[1]].image_ * F.data[term_index[2]].image_;*/

	//image_part *= real_part;
	//image_part *= 2.0;

	//real_part *= real_part;
}

template <class TYPE>
void TypeTetElementMaterial<TYPE>::derivativeI1(lobo::LoboComplexMatrix3<LoboComplext, TYPE> &F, TYPE& real_part, TYPE&image_part, bool imageonly)
{
	/*real_part = F.data[0].real_*F.data[0].real_ + F.data[4].real_ * F.data[4].real_
		+ F.data[8].real_*F.data[8].real_;*/
	real_part = 0;
	image_part = 0;

	for (int i = 0;i < 9;i++)
	{
		image_part += F.data[i].real_*F.data[i].image_;
	}

	if(!imageonly)
	for (int i = 0;i < 9;i++)
	{
		real_part += F.data[i].real_*F.data[i].real_;
	}

	/*image_part =
		F.data[0].image_*F.data[0].real_ + F.data[4].image_*F.data[4].real_
		+F.data[8].image_*F.data[8].real_;*/
	image_part *= 2.0;
}

template <class TYPE>
void TypeTetElementMaterial<TYPE>::secondDerviativeI1(lobo::LoboComplexMatrix3<LoboComplexDualt, TYPE> &F, TYPE& real_part, TYPE&image_part)
{
	/*image_part = F.data[0]b*F.data[0]d*2.0 + F.data[4]b*F.data[4]d*2.0 + F.data[8]b*F.data[8]d*2.0;*/

	image_part = 0;
	for (int i = 0;i < 9;i++)
	{
		image_part += F.data[i]b*F.data[i]d;
	}
	image_part *= 2.0;
}


template <class TYPE>
void TypeTetElementMaterial<TYPE>::secondDerivativeFdet(lobo::LoboComplexMatrix3<LoboComplexDualt, TYPE> &F, TYPE& real_part, TYPE&image_part, bool imageonly)
{
	//============ term1 =============
	TYPE term1_real_part = 0;
	TYPE term1_image_part = 0;
	std::vector<int> term1_index;
	term1_index.resize(3);
	term1_index[0] = 2;
	term1_index[1] = 4;
	term1_index[2] = 6;
	this->secondDerivativeFdetThreeTerm(term1_index,  F, term1_real_part, term1_image_part, imageonly);

	//============ term2 =============
	term1_index[0] = 1;
	term1_index[1] = 5;
	term1_index[2] = 6;
	TYPE term2_real_part = 0;
	TYPE term2_image_part = 0;
	this->secondDerivativeFdetThreeTerm(term1_index,  F, term2_real_part, term2_image_part, imageonly);


	//============ term3 =============
	term1_index[0] = 2;
	term1_index[1] = 3;
	term1_index[2] = 7;
	TYPE term3_real_part = 0;
	TYPE term3_image_part = 0;
	this->secondDerivativeFdetThreeTerm(term1_index,  F, term3_real_part, term3_image_part, imageonly);

	//============ term4 =============
	term1_index[0] = 0;
	term1_index[1] = 5;
	term1_index[2] = 7;
	TYPE term4_real_part = 0;
	TYPE term4_image_part = 0;
	this->secondDerivativeFdetThreeTerm(term1_index,  F, term4_real_part, term4_image_part, imageonly);

	//============ term5 =============
	term1_index[0] = 1;
	term1_index[1] = 3;
	term1_index[2] = 8;
	TYPE term5_real_part = 0;
	TYPE term5_image_part = 0;
	this->secondDerivativeFdetThreeTerm(term1_index,  F, term5_real_part, term5_image_part, imageonly);

	//============ term5 =============
	term1_index[0] = 0;
	term1_index[1] = 4;
	term1_index[2] = 8;
	TYPE term6_real_part = 0;
	TYPE term6_image_part = 0;
	this->secondDerivativeFdetThreeTerm(term1_index,  F, term6_real_part, term6_image_part, imageonly);

	if (!imageonly)
	{
		real_part = -term1_real_part + term2_real_part + term3_real_part - term4_real_part - term5_real_part
			+ term6_real_part;
	}

	image_part = -term1_image_part + term2_image_part + term3_image_part - term4_image_part - term5_image_part + term6_image_part;

}

template <class TYPE>
void TypeTetElementMaterial<TYPE>::derivativeFdet(int r,  const lobo::LoboComplexMatrix3<LoboComplext, TYPE> &F, TYPE& real_part, TYPE&image_part, bool imageonly)
{
	TYPE term1_real_part = 0;
	TYPE term1_image_part = 0;
	std::vector<int> term1_index;
	term1_index.resize(3);
	term1_index[0] = 2;
	term1_index[1] = 4;
	term1_index[2] = 6;

	this->derivativeFdetThreeTerm(term1_index, F, term1_real_part, term1_image_part, imageonly);

	term1_index[0] = 1;
	term1_index[1] = 5;
	term1_index[2] = 6;
	TYPE term2_real_part = 0;
	TYPE term2_image_part = 0;

	this->derivativeFdetThreeTerm(term1_index, F, term2_real_part, term2_image_part, imageonly);

	//this->derivativeFdetSixTerm(term2_index, F, term2_real_part, term2_image_part);

	//============ term3 =============
	term1_index[0] = 2;
	term1_index[1] = 3;
	term1_index[2] = 7;
	TYPE term3_real_part = 0;
	TYPE term3_image_part = 0;
	this->derivativeFdetThreeTerm(term1_index, F, term3_real_part, term3_image_part, imageonly);


	//============ term4 =============
	term1_index[0] = 0;
	term1_index[1] = 5;
	term1_index[2] = 7;
	TYPE term4_real_part = 0;
	TYPE term4_image_part = 0;
	this->derivativeFdetThreeTerm(term1_index, F, term4_real_part, term4_image_part, imageonly);


	//============ term5 =============
	term1_index[0] = 1;
	term1_index[1] = 3;
	term1_index[2] = 8;
	TYPE term5_real_part = 0;
	TYPE term5_image_part = 0;
	this->derivativeFdetThreeTerm(term1_index, F, term5_real_part, term5_image_part, imageonly);

	//============ term6 =============
	term1_index[0] = 0;
	term1_index[1] = 4;
	term1_index[2] = 8;
	TYPE term6_real_part = 0;
	TYPE term6_image_part = 0;
	this->derivativeFdetThreeTerm(term1_index, F, term6_real_part, term6_image_part, imageonly);

	if (!imageonly)
	{
		real_part = -term1_real_part + term2_real_part + term3_real_part - term4_real_part - term5_real_part
			+ term6_real_part;
	}

	image_part = -term1_image_part + term2_image_part + term3_image_part - term4_image_part - term5_image_part+ term6_image_part;
}

template <class TYPE>
void TypeTetElementMaterial<TYPE>::derivativeE(lobo::LoboComplexMatrix3<LoboComplext, TYPE> &F, lobo::LoboComplexMatrix3<LoboComplext, TYPE> &E)
{
	E.data[0].image_ = F.data[0].real_*F.data[0].image_ + F.data[1].real_*F.data[1].image_ + F.data[2].real_*F.data[2].image_ - 1.0;
	//E.data[0].image_ *= 2.0;
	
	E.data[3].image_ = F.data[0].image_*F.data[3].real_ + F.data[0].real_*F.data[3].image_
		+ F.data[1].image_*F.data[4].real_ + F.data[1].real_*F.data[4].image_
		+ F.data[2].image_*F.data[5].real_ + F.data[2].real_*F.data[5].image_;

	E.data[6].image_ = F.data[0].image_*F.data[6].real_ + F.data[0].real_*F.data[6].image_
		+ F.data[1].image_*F.data[7].real_ + F.data[1].real_*F.data[7].image_
		+ F.data[2].image_*F.data[8].real_ + F.data[2].real_*F.data[8].image_;
	
	E.data[1].image_ = E.data[3].image_;
	E.data[2].image_ = E.data[6].image_;

	E.data[4].image_ = F.data[3].real_*F.data[3].image_ + F.data[4].real_*F.data[4].image_ + F.data[5].real_*F.data[5].image_ - 1.0;
	//E.data[4].image_ *= 2.0;

	E.data[5].image_ = F.data[3].image_*F.data[6].real_ + F.data[3].real_*F.data[6].image_
		+ F.data[4].image_*F.data[7].real_ + F.data[4].real_*F.data[7].image_
		+ F.data[5].image_*F.data[8].real_ + F.data[5].real_*F.data[8].image_;

	E.data[7].image_ = E.data[5].image_;


	E.data[8].image_ = F.data[6].real_*F.data[6].image_ + F.data[7].real_*F.data[7].image_ + F.data[8].real_*F.data[8].image_ - 1.0;
	//E.data[8].image_ *= 2.0;

	E.data[1].image_ *= 0.5;
	E.data[2].image_ *= 0.5;
	E.data[3].image_ *= 0.5;
	E.data[5].image_ *= 0.5;
	E.data[6].image_ *= 0.5;
	E.data[7].image_ *= 0.5;
}

template <class TYPE>
void TypeTetElementMaterial<TYPE>::derivativeI2(lobo::LoboComplexMatrix3<LoboComplext, TYPE> &F, TYPE& real_part, TYPE&image_part, bool real_part_need)
{
	TYPE FtF[9];
	FtF[0] = F.data[0].real_*F.data[0].real_ + F.data[1].real_*F.data[1].real_ + F.data[2].real_*F.data[2].real_;
	//E.data[0].image_ *= 2.0;

	FtF[3] = F.data[0].real_*F.data[3].real_ 
		+ F.data[1].real_*F.data[4].real_ 
		+ F.data[2].real_*F.data[5].real_ ;

	FtF[6] = F.data[0].real_*F.data[6].real_ 
		+ F.data[1].real_*F.data[7].real_ 
		+ F.data[2].real_*F.data[8].real_ ;

	FtF[1] = FtF[3];
	FtF[2] = FtF[6];

	FtF[4] = F.data[3].real_*F.data[3].real_ + F.data[4].real_*F.data[4].real_ + F.data[5].real_*F.data[5].real_;
	//E.data[4].image_ *= 2.0;

	FtF[5] = F.data[3].real_*F.data[6].real_ 
		+ F.data[4].real_*F.data[7].real_ 
		+ F.data[5].real_*F.data[8].real_ ;

	FtF[7] = FtF[5];

	FtF[8] = F.data[6].real_*F.data[6].real_ + F.data[7].real_*F.data[7].real_ + F.data[8].real_*F.data[8].real_;
	image_part = 0;

	for (int i = 0;i < 3;i++)
	{
		for (int j = 0;j < 3;j++)
		{
			for (int k = 0;k < 3;k++)
			{
				image_part += 4.0*F.data[k * 3 + i].real_*FtF[j * 3 + k] * F.data[j * 3 + i].image_;
			}
		}
	}

	if (real_part_need)
	{
		real_part = 0;
		for (int i = 0;i < 9;i++)
		{
			real_part += FtF[i] * FtF[i];
		}
	}
}

template <class TYPE>
void TypeTetElementMaterial<TYPE>::secondDerivativeI2(lobo::LoboComplexMatrix3<LoboComplexDualt, TYPE> &F, TYPE& real_part, TYPE&image_part, bool real_part_need /*= false*/)
{
	TYPE FtF[9];
	FtF[0] = F.data[0]re*F.data[0]re + F.data[1]re*F.data[1]re + F.data[2]re*F.data[2]re;
	//E.data[0].image_ *= 2.0;

	FtF[3] = F.data[0]re*F.data[3]re
		+ F.data[1]re*F.data[4]re
		+ F.data[2]re*F.data[5]re;

	FtF[6] = F.data[0]re*F.data[6]re
		+ F.data[1]re*F.data[7]re
		+ F.data[2]re*F.data[8]re;

	FtF[1] = FtF[3];
	FtF[2] = FtF[6];

	FtF[4] = F.data[3]re*F.data[3]re + F.data[4]re*F.data[4]re + F.data[5]re*F.data[5]re;
	//E.data[4].image_ *= 2.0;

	FtF[5] = F.data[3]re*F.data[6]re
		+ F.data[4]re*F.data[7]re
		+ F.data[5]re*F.data[8]re;

	FtF[7] = FtF[5];

	FtF[8] = F.data[6]re*F.data[6]re + F.data[7]re*F.data[7]re + F.data[8]re*F.data[8]re;




	

	TYPE FvtF[9];
	FvtF[0] = F.data[0]d*F.data[0]re + F.data[1]d*F.data[1]re + F.data[2]d*F.data[2]re;
	//E.data[0].image_ *= 2.0;

	FvtF[3] = F.data[0]d*F.data[3]re
		+ F.data[1]d*F.data[4]re
		+ F.data[2]d*F.data[5]re;

	FvtF[6] = F.data[0]d*F.data[6]re
		+ F.data[1]d*F.data[7]re
		+ F.data[2]d*F.data[8]re;

	FvtF[1] = F.data[0]re*F.data[3]d
		+ F.data[1]re*F.data[4]d
		+ F.data[2]re*F.data[5]d;
	FvtF[2] = F.data[0]re*F.data[6]d
		+ F.data[1]re*F.data[7]d
		+ F.data[2]re*F.data[8]d;

	FvtF[4] = F.data[3]d*F.data[3]re + F.data[4]d*F.data[4]re + F.data[5]d*F.data[5]re;
	//E.data[4].image_ *= 2.0;

	FvtF[5] = F.data[3]d*F.data[6]re
		+ F.data[4]d*F.data[7]re
		+ F.data[5]d*F.data[8]re;

	FvtF[7] = F.data[3]re*F.data[6]d
		+ F.data[4]re*F.data[7]d
		+ F.data[5]re*F.data[8]d;

	FvtF[8] = F.data[6]d*F.data[6]re + F.data[7]d*F.data[7]re + F.data[8]d*F.data[8]re;


	image_part = 0;
	for (int i = 0;i < 3;i++)
	{
		for (int j = 0;j < 3;j++)
		{
			for (int k = 0;k < 3;k++)
			{
				image_part += 4.0*F.data[k * 3 + i]d*FtF[j * 3 + k] * F.data[j * 3 + i]b;
			}
		}
	}

	for (int i = 0;i < 3;i++)
	{
		for (int j = 0;j < 3;j++)
		{
			for (int k = 0;k < 3;k++)
			{
				image_part += 4.0*F.data[k * 3 + i]re*FvtF[j * 3 + k] * F.data[j * 3 + i]b;
			}
		}
	}

	for (int i = 0;i < 3;i++)
	{
		for (int j = 0;j < 3;j++)
		{
			for (int k = 0;k < 3;k++)
			{
				image_part += 4.0*F.data[k * 3 + i]re*FvtF[k * 3 + j] * F.data[j * 3 + i]b;
			}
		}
	}

	for (int i = 0;i < 3;i++)
	{
		for (int j = 0;j < 3;j++)
		{
			for (int k = 0;k < 3;k++)
			{
				image_part += 4.0*F.data[k * 3 + i]re*FtF[j * 3 + k] * F.data[j * 3 + i]uv;
			}
		}
	}

}

template <class TYPE>
void TypeTetElementMaterial<TYPE>::getFComplex(int elementIndex, Matrix3&F, lobo::LoboComplexMatrix3<LoboComplexDualt, TYPE> &F_complex_output, int u, int v)
{
	int index = (1 + u)*(u) / 2+v;
	F_complex_output = element_dF_dudv[elementIndex * 78 + index];
	F_complex_output.setRealValue(F.data());

}

template <class TYPE>
void TypeTetElementMaterial<TYPE>::getFComplex_single(int elementIndex, Matrix3&F, lobo::LoboComplexMatrix3<LoboComplext, TYPE> &F_complex_output, int u)
{
	F_complex_output = element_dF_du[elementIndex * 12 + u];
	F_complex_output.setRealValue(F.data());
}

#undef d
#undef b
#undef re
#undef uv