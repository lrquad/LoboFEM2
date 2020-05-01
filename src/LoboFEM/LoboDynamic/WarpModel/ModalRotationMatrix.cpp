#include "ModalRotationMatrix.h"
#include "LoboVolumtricMesh/LoboTetMesh.h"
#include "Functions/SkewMatrix.h"

typedef Eigen::Triplet<double> EIGEN_TRI;

Lobo::ModalRotationMatrix::ModalRotationMatrix(Lobo::LoboTetMesh *tetmesh_)
{
    tetmesh = tetmesh_;
    w_operator = new Eigen::SparseMatrix<double>();
    tetmesh->getNodeElements(node_elements);
}

Lobo::ModalRotationMatrix::~ModalRotationMatrix()
{
    delete w_operator;
}

void Lobo::ModalRotationMatrix::computeModalRotationSparseMatrix_W(Eigen::SparseMatrix<double> *W)
{
    int R = tetmesh->getNumVertex() * 3;
    W->resize(R, R);
    int numElements = tetmesh->getNumElements();

    Eigen::MatrixXd ele_W(3, 12);
    ele_W.setZero();

    Eigen::Matrix3d skewMatrix_v;

    std::vector<EIGEN_TRI> tri_entry_;

    for (int i = 0; i < numElements; i++)
    {
        Lobo::TetElementData *ele = tetmesh->getTetElement(i);
        for (int j = 0; j < 4; j++)
        {
            Eigen::Vector3d v = ele->Phi_derivate.row(j);
            skewMatrix(v, skewMatrix_v);
            ele_W.block(0, j * 3, 3, 3) = skewMatrix_v;
        }
        ele_W /= 2.0;

        for (int j = 0; j < 4; j++)
        {
            int nodeid = tetmesh->tet_indices.data()[i * 4 + j];
            int has_elements = node_elements[nodeid].size();

            for (int l = 0; l < 3; l++)
            {
                for (int m = 0; m < 12; m++)
                {
                    int NID = tetmesh->tet_indices.data()[i * 4 + m / 3];
                    int row = nodeid * 3 + l;
                    int col = NID * 3 + m % 3;
                    double value = ele_W.data()[m * 3 + l] / has_elements;
                    tri_entry_.push_back(EIGEN_TRI(row, col, value));
                }
            }
        }
    }
    W->setFromTriplets(tri_entry_.begin(), tri_entry_.end());
}

void Lobo::ModalRotationMatrix::computeWarpingRotationVector_subnodes(Eigen::VectorXd &u, Eigen::VectorXd &w, std::vector<int> &nodelist)
{
    int numVertex = tetmesh->getNumVertex();
	int R = numVertex * 3;
	double valuebuffer[3];
	for (int j = 0; j < nodelist.size(); j++)
	{
		int i = nodelist[j];
		Eigen::Vector3d ww;
		ww.data()[0] = (double)w.data()[i * 3 + 0];
		ww.data()[1] = (double)w.data()[i * 3 + 1];
		ww.data()[2] = (double)w.data()[i * 3 + 2];

		double wwNorm = ww.norm();

		Eigen::Vector3d w_hat = ww.normalized();
		if (std::abs(wwNorm) < 1e-5)
		{
			wwNorm += 1;
			w_hat = ww / wwNorm;
		}

		double cs = (1 - std::cos(wwNorm)) / wwNorm;
		double sn = 1 - std::sin(wwNorm) / wwNorm;

		Eigen::Matrix3d w_hat_crossMat;
		skewMatrix(w_hat, w_hat_crossMat);

		Eigen::Matrix3d R_i;
		R_i.setIdentity();
		R_i += w_hat_crossMat*cs + w_hat_crossMat*w_hat_crossMat*sn;

		/*AngleAxis<double> aa(wwNorm, w_hat);
		R_i = aa.toRotationMatrix();*/

		for (int r = 0;r < 3;r++)
		{
			valuebuffer[r] = 0;
			for (int c = 0;c < 3;c++)
			{
				valuebuffer[r] += R_i.data()[c * 3 + r] * u.data()[i * 3 + c];
			}
		}
		for (int r = 0;r < 3;r++)
		{
			u.data()[i * 3 + r] = valuebuffer[r];
		}
	}
}

void Lobo::ModalRotationMatrix::computeLocalOrientationVector(Eigen::VectorXd &f, const Eigen::VectorXd &w, bool transpose)
{
    int numVertex = tetmesh->getNumVertex();
	int R = numVertex * 3;
	double valuebuffer[3];
	for (int i = 0; i < numVertex; i++)
	{
		Eigen::Vector3d ww;
		ww.data()[0] = (double)w.data()[i * 3 + 0];
		ww.data()[1] = (double)w.data()[i * 3 + 1];
		ww.data()[2] = (double)w.data()[i * 3 + 2];

		double wwNorm = ww.norm();

		Eigen::Vector3d w_hat = ww.normalized();
		if (std::abs(wwNorm) < 1e-15)
		{
			w_hat.setZero();
		}

		double cs = (1 - std::cos(wwNorm));
		double sn = std::sin(wwNorm);

		Eigen::Matrix3d w_hat_crossMat;
		skewMatrix(w_hat, w_hat_crossMat);

		Eigen::Matrix3d R_i;
		R_i.setIdentity();
		R_i += w_hat_crossMat*sn + w_hat_crossMat*w_hat_crossMat*cs;

		if (transpose)
		{
			for (int r = 0;r < 3;r++)
			{
				valuebuffer[r] = 0;
				for (int c = 0;c < 3;c++)
				{
					valuebuffer[r] += R_i.data()[r * 3 + c] * f.data()[i * 3 + c];
				}
			}
		}
		else
		{
			for (int r = 0;r < 3;r++)
			{
				valuebuffer[r] = 0;
				for (int c = 0;c < 3;c++)
				{
					valuebuffer[r] += R_i.data()[c * 3 + r] * f.data()[i * 3 + c];
				}
			}
		}

		for (int r = 0;r < 3;r++)
		{
			f.data()[i * 3 + r] = valuebuffer[r];
		}
	}
}