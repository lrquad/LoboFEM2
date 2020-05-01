#include "TypeMaterialWithCompressionResistance.h"

LOBO_TEMPLATE_INSTANT(TypeMaterialWithCompressionResistance)

template <class TYPE>
TypeMaterialWithCompressionResistance<TYPE>::TypeMaterialWithCompressionResistance(bool enableCompressionResistance_ /*= 0*/)
{
	enableCompressionResistance = enableCompressionResistance_;
}

template <class TYPE>
TypeMaterialWithCompressionResistance<TYPE>::~TypeMaterialWithCompressionResistance()
{

}

template <class TYPE>
TYPE TypeMaterialWithCompressionResistance<TYPE>::GetCompressionResistanceFactor(int elementIndex)
{
	return 1.0;
}

template <class TYPE>
void TypeMaterialWithCompressionResistance<TYPE>::AddCompressionResistanceGradient(int elementIndex, TYPE * invariants, TYPE * gradient)
{
	if (enableCompressionResistance)
	{
		TYPE IIIC = invariants[2];
		TYPE J = std::sqrt(IIIC);

		if (J < 1)
		{
			TYPE compressionResistanceFactor = GetCompressionResistanceFactor(elementIndex);

			gradient[2] += -compressionResistanceFactor * (J - 1.0) * (J - 1.0) / (1728.0 * J);
		}
	}
}

template <class TYPE>
void TypeMaterialWithCompressionResistance<TYPE>::AddCompressionResistanceHessian(int elementIndex, TYPE * invariants, TYPE * hessian)
{
	if (enableCompressionResistance)
	{
		TYPE IIIC = invariants[2];
		TYPE J = sqrt(IIIC);

		if (J < 1.0)
		{
			TYPE compressionResistanceFactor = GetCompressionResistanceFactor(elementIndex);
			hessian[5] += compressionResistanceFactor * (1.0 - J) * (1.0 + J) / (3456.0 * J * J * J);
		}
	}
}

template <class TYPE>
void TypeMaterialWithCompressionResistance<TYPE>::AddCompressionResistanceEnergy(int elementIndex, TYPE * invariants, TYPE * energy)
{
	if (enableCompressionResistance)
	{
		TYPE IIIC = invariants[2];
		TYPE J = std::sqrt(IIIC);

		if (J < 1)
		{
			TYPE compressionResistanceFactor = GetCompressionResistanceFactor(elementIndex);
			*energy += -compressionResistanceFactor * (J - 1.0) * (J - 1.0) * (J - 1.0) / 2592.0;
		}
	}
}
