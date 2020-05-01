#include "computeSVDMatrix3d.h"
#include "Vega/libraries/minivector/eig3.h"
using namespace Eigen;

namespace Lobo
{
void computeSVD(Matrix3d &F, Matrix3d &U, Matrix3d &V, Matrix3d &singularF, double singularValue_eps, int modifiedSVD)
{
	singularF.setZero();
	Matrix3d normalEq = F.transpose() * F;
	Vector3d eigenValues;
	Vector3d eigenVectors[3];
	eigen_sym(normalEq, eigenValues, eigenVectors);
	V << eigenVectors[0][0], eigenVectors[1][0], eigenVectors[2][0],
		eigenVectors[0][1], eigenVectors[1][1], eigenVectors[2][1],
		eigenVectors[0][2], eigenVectors[1][2], eigenVectors[2][2];

	if (V.determinant() < 0.0)
	{
		V.data()[0] *= -1.0;
		V.data()[1] *= -1.0;
		V.data()[2] *= -1.0;
	}

	singularF.data()[0 * 3 + 0] = (eigenValues[0] > 0.0) ? sqrt(eigenValues[0]) : 0.0;
	singularF.data()[1 * 3 + 1] = (eigenValues[1] > 0.0) ? sqrt(eigenValues[1]) : 0.0;
	singularF.data()[2 * 3 + 2] = (eigenValues[2] > 0.0) ? sqrt(eigenValues[2]) : 0.0;

	Vector3d singualrFInvers;
	singualrFInvers[0] = (singularF.data()[0 * 3 + 0] > singularValue_eps) ? (1.0 / singularF.data()[0 * 3 + 0]) : 0.0;
	singualrFInvers[1] = (singularF.data()[1 * 3 + 1] > singularValue_eps) ? (1.0 / singularF.data()[1 * 3 + 1]) : 0.0;
	singualrFInvers[2] = (singularF.data()[2 * 3 + 2] > singularValue_eps) ? (1.0 / singularF.data()[2 * 3 + 2]) : 0.0;
	U = F * V * singualrFInvers.asDiagonal();
	if ((singularF.data()[0 * 3 + 0] < singularValue_eps) && (singularF.data()[1 * 3 + 1] < singularValue_eps) && (singularF.data()[2 * 3 + 2] < singularValue_eps))
	{
		U.setIdentity();
	}
	else
	{
		int done = 0;
		for (int dim = 0; dim < 3; dim++)
		{
			int dimA = dim;
			int dimB = (dim + 1) % 3;
			int dimC = (dim + 2) % 3;
			if ((singularF.data()[dimB * 3 + dimB] < singularValue_eps) && (singularF.data()[dimC * 3 + dimC] < singularValue_eps))
			{
				Vector3d temVec1(U.data()[dimA * 3 + 0], U.data()[dimA * 3 + 1], U.data()[dimA * 3 + 2]);
				Vector3d temVec2;
				findOrthonormalVector(temVec1, temVec2);
				Vector3d temVec3 = temVec1.cross(temVec2).normalized();
				U.data()[dimB * 3 + 0] = temVec2[0];
				U.data()[dimB * 3 + 1] = temVec2[1];
				U.data()[dimB * 3 + 2] = temVec2[2];
				U.data()[dimC * 3 + 0] = temVec3[0];
				U.data()[dimC * 3 + 1] = temVec3[1];
				U.data()[dimC * 3 + 2] = temVec3[2];
				if (U.determinant() < 0.0)
				{
					U.data()[dimB * 3 + 0] *= -1.0;
					U.data()[dimB * 3 + 1] *= -1.0;
					U.data()[dimB * 3 + 2] *= -1.0;
				}
				done = 1;
				break;
			}
		}

		if (!done)
		{
			for (int dim = 0; dim < 3; dim++)
			{
				int dimA = dim;
				int dimB = (dim + 1) % 3;
				int dimC = (dim + 2) % 3;

				if (singularF.data()[dimA * 3 + dimA] < singularValue_eps)
				{
					// columns dimB and dimC are both good, but column dimA corresponds to a tiny singular value
					Vector3d tmpVec1(U.data()[dimB * 3 + 0], U.data()[dimB * 3 + 1], U.data()[dimB * 3 + 2]); // column dimB
					Vector3d tmpVec2(U.data()[dimC * 3 + 0], U.data()[dimC * 3 + 1], U.data()[dimC * 3 + 2]); // column dimC
					Vector3d tmpVec3 = tmpVec1.cross(tmpVec2).normalized();
					U.data()[dimA * 3 + 0] = tmpVec3[0];
					U.data()[dimA * 3 + 1] = tmpVec3[1];
					U.data()[dimA * 3 + 2] = tmpVec3[2];
					if (U.determinant() < 0.0)
					{
						U.data()[dimA * 3 + 0] *= -1.0;
						U.data()[dimA * 3 + 1] *= -1.0;
						U.data()[dimA * 3 + 2] *= -1.0;
					}
					done = 1;
					break; // out of for
				}
			}
		}

		//Original code need to set a flag modeifiedSVD, our simulator is only for
		//invertible, so I removed the flag.
		if ((!done) && (modifiedSVD == 1))
		{
			if (U.determinant() < 0.0)
			{
				int smallestSingularValueIndex = 0;
				for (int dim = 1; dim < 3; dim++)
				{
					if (singularF.data()[dim * 3 + dim] < singularF.data()[smallestSingularValueIndex * 3 + smallestSingularValueIndex])
					{
						smallestSingularValueIndex = dim;
					}
				}

				singularF.data()[smallestSingularValueIndex * 3 + smallestSingularValueIndex] *= -1.0;
				U.data()[smallestSingularValueIndex * 3 + 0] *= -1.0;
				U.data()[smallestSingularValueIndex * 3 + 1] *= -1.0;
				U.data()[smallestSingularValueIndex * 3 + 2] *= -1.0;
			}
			//end of if
		}
		//end of else
	}
}

void eigen_sym(Matrix3d &a, Vector3d &eig_val, Vector3d (&eig_vec)[3])
{
	double A[3][3] = {{a.data()[0], a.data()[3], a.data()[6]},
					  {a.data()[1], a.data()[4], a.data()[7]},
					  {a.data()[2], a.data()[5], a.data()[8]}};

	double V[3][3];
	double d[3];
	eigen_decomposition(A, V, d);

	eig_val = Vector3d(d[2], d[1], d[0]);

	eig_vec[0] = Vector3d(V[0][2], V[1][2], V[2][2]);
	eig_vec[1] = Vector3d(V[0][1], V[1][1], V[2][1]);
	eig_vec[2] = Vector3d(V[0][0], V[1][0], V[2][0]);
}

void findOrthonormalVector(Vector3d &v1, Vector3d &v2)
{
	int smallestIndex = 0;
	for (int dim = 1; dim < 3; dim++)
	{
		if (fabs(v1[dim]) < fabs(v1[smallestIndex]))
		{
			smallestIndex = dim;
		}
	}

	Vector3d axis(0.0, 0.0, 0.0);
	axis[smallestIndex] = 1.0;
	v2 = v1.cross(axis).normalized();
}
} // namespace Lobo