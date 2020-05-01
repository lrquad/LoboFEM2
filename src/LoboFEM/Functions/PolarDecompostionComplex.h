#pragma once
#include "AutoDiff/LoboComplex.h"
#include <vector>

//Changed from Vega FEM

namespace lobo {

	template<class COMPLEX_TYPE, class TYPE>
	COMPLEX_TYPE oneNorm(const COMPLEX_TYPE* A)
	{
		COMPLEX_TYPE norm;
		for (int i = 0;i < 3;i++)
		{
			COMPLEX_TYPE columnAbsSum = lobo::abs(A[i*3 + 0]) + abs(A[i*3 + 1]) + abs(A[i*3 + 2]);
			if (lobo::largerReal(columnAbsSum, norm))
			{
				norm = columnAbsSum;
			}
		}
		return norm;
	}

	template<class COMPLEX_TYPE, class TYPE>
	COMPLEX_TYPE infNorm(const COMPLEX_TYPE* A)
	{
		COMPLEX_TYPE norm;
		for (int i = 0;i < 3;i++)
		{
			COMPLEX_TYPE rowAbsSum = lobo::abs(A[i + 0]) + abs(A[i + 3]) + abs(A[i + 6]);
			if (lobo::largerReal(rowAbsSum, norm))
			{
				norm = rowAbsSum;
			}
		}
		return norm;
	}

	template<class COMPLEX_TYPE, class TYPE>
	inline void crossProduct(const COMPLEX_TYPE * a, const COMPLEX_TYPE * b, COMPLEX_TYPE * c)
	{
		c[0] = a[3] * b[6] - a[6] * b[3];
		c[3] = a[6] * b[0] - a[0] * b[6];
		c[6] = a[0] * b[3] - a[3] * b[0];
	}

	template<class COMPLEX_TYPE, class TYPE>
	TYPE ComputePolarDecomposition(const COMPLEX_TYPE * M, COMPLEX_TYPE * Q, COMPLEX_TYPE * S, TYPE tolerance, int forceRotation = 0)
	{
		COMPLEX_TYPE Mk[9];
		COMPLEX_TYPE Ek[9];
		COMPLEX_TYPE det, M_oneNorm, M_infNorm, E_oneNorm;

		// Mk = M^T
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				Mk[3 * i + j] = M[3 * j + i];

		M_oneNorm = oneNorm<COMPLEX_TYPE, TYPE>(Mk);
		M_infNorm = infNorm<COMPLEX_TYPE, TYPE>(Mk);
		COMPLEX_TYPE rhs = M_oneNorm * tolerance;

		do
		{
			COMPLEX_TYPE MadjTk[9];

			// row 2 x row 3
			crossProduct<COMPLEX_TYPE,TYPE>(&(Mk[1]), &(Mk[2]), &(MadjTk[0]));
			// row 3 x row 1
			crossProduct<COMPLEX_TYPE, TYPE>(&(Mk[2]), &(Mk[0]), &(MadjTk[1]));
			// row 1 x row 2
			crossProduct<COMPLEX_TYPE, TYPE>(&(Mk[0]), &(Mk[1]), &(MadjTk[2]));

			det = Mk[0] * MadjTk[0] + Mk[3] * MadjTk[3] + Mk[6] * MadjTk[6];
			

			if (det == (TYPE)0.0)
			{
				printf("Warning (polarDecomposition) : zero determinant encountered.\n");
				break;
			}

			COMPLEX_TYPE MadjT_one = oneNorm<COMPLEX_TYPE, TYPE>(MadjTk);
			COMPLEX_TYPE MadjT_inf = infNorm<COMPLEX_TYPE, TYPE>(MadjTk);

			//COMPLEX_TYPE gamma = lobo::sqrt(lobo::sqrt((MadjT_one * MadjT_inf) / (M_oneNorm * M_infNorm * det * det)));

			COMPLEX_TYPE tmp = (MadjT_one * MadjT_inf) / (M_oneNorm * M_infNorm * det * det);
			COMPLEX_TYPE gamma = lobo::pow(tmp, (TYPE)0.25);

			COMPLEX_TYPE g1 = gamma * (TYPE)0.5;
			COMPLEX_TYPE g2 = (TYPE)0.5 / (gamma * det);

			for (int i = 0; i < 9; i++)
			{
				Ek[i] = Mk[i];
				Mk[i] = g1 * Mk[i] + g2 * MadjTk[i];
				Ek[i] -= Mk[i];
			}

			E_oneNorm = oneNorm<COMPLEX_TYPE, TYPE>(Ek);
			M_oneNorm = oneNorm<COMPLEX_TYPE, TYPE>(Mk);
			M_infNorm = infNorm<COMPLEX_TYPE, TYPE>(Mk);    
			rhs = M_oneNorm * tolerance;
		} while (lobo::largerReal(E_oneNorm,rhs));

		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				Q[3 * i + j] = Mk[3 * j + i];

		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
			{
				S[3 * j + i] *= (TYPE)0.0;
				for (int k = 0; k < 3; k++)
					S[3 * j + i] += Mk[3 * k + i] * M[3 * j + k];
			}


		S[1] = S[3] = (TYPE)0.5 * (S[1] + S[3]);
		S[2] = S[6] = (TYPE)0.5 * (S[2] + S[6]);
		S[5] = S[7] = (TYPE)0.5 * (S[5] + S[7]);

		return 0;
	}


	template<class COMPLEX_TYPE, class TYPE>
	TYPE ComputePolarDecompositionV2(const COMPLEX_TYPE * M, COMPLEX_TYPE * Q, COMPLEX_TYPE * S, TYPE tolerance, int forceRotation = 0)
	{
		COMPLEX_TYPE Mk[9];
		COMPLEX_TYPE Ek[9];
		COMPLEX_TYPE det, M_oneNorm, M_infNorm, E_oneNorm;

		// Mk = M^T
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				Mk[3 * i + j] = M[3 * j + i];

		M_oneNorm = oneNorm<COMPLEX_TYPE, TYPE>(Mk);
		M_infNorm = infNorm<COMPLEX_TYPE, TYPE>(Mk);
		COMPLEX_TYPE rhs = M_oneNorm * tolerance;

		do
		{
			COMPLEX_TYPE MadjTk[9];

			// row 2 x row 3
			crossProduct<COMPLEX_TYPE, TYPE>(&(Mk[1]), &(Mk[2]), &(MadjTk[0]));
			// row 3 x row 1
			crossProduct<COMPLEX_TYPE, TYPE>(&(Mk[2]), &(Mk[0]), &(MadjTk[1]));
			// row 1 x row 2
			crossProduct<COMPLEX_TYPE, TYPE>(&(Mk[0]), &(Mk[1]), &(MadjTk[2]));

			det = Mk[0] * MadjTk[0] + Mk[3] * MadjTk[3] + Mk[6] * MadjTk[6];


			if (det == (TYPE)0.0)
			{
				printf("Warning (polarDecomposition) : zero determinant encountered.\n");
				break;
			}

			COMPLEX_TYPE MadjT_one = oneNorm<COMPLEX_TYPE, TYPE>(MadjTk);
			COMPLEX_TYPE MadjT_inf = infNorm<COMPLEX_TYPE, TYPE>(MadjTk);

			//COMPLEX_TYPE gamma = lobo::sqrt(lobo::sqrt((MadjT_one * MadjT_inf) / (M_oneNorm * M_infNorm * det * det)));

			COMPLEX_TYPE tmp = (MadjT_one * MadjT_inf) / (M_oneNorm * M_infNorm * det * det);
			COMPLEX_TYPE gamma = lobo::pow(tmp, (TYPE)0.25);

			COMPLEX_TYPE g1 = gamma * (TYPE)0.5;
			COMPLEX_TYPE g2 = (TYPE)0.5 / (gamma * det);

			for (int i = 0; i < 9; i++)
			{
				Ek[i] = Mk[i];
				Mk[i] = g1 * Mk[i] + g2 * MadjTk[i];
				Ek[i] -= Mk[i];
			}

			E_oneNorm = oneNorm<COMPLEX_TYPE, TYPE>(Ek);
			M_oneNorm = oneNorm<COMPLEX_TYPE, TYPE>(Mk);
			M_infNorm = infNorm<COMPLEX_TYPE, TYPE>(Mk);
			rhs = M_oneNorm * tolerance;
		} while (lobo::largerReal(E_oneNorm, rhs));

		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				Q[3 * i + j] = Mk[3 * j + i];

		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
			{
				S[3 * j + i] *= (TYPE)0.0;
				for (int k = 0; k < 3; k++)
					S[3 * j + i] += Mk[3 * k + i] * M[3 * j + k];
			}


		S[1] = S[3] = (TYPE)0.5 * (S[1] + S[3]);
		S[2] = S[6] = (TYPE)0.5 * (S[2] + S[6]);
		S[5] = S[7] = (TYPE)0.5 * (S[5] + S[7]);

		return 0;
	}


}