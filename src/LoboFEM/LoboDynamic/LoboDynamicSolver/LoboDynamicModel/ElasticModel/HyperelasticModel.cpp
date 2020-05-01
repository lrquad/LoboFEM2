#include "HyperelasticModel.h"
#include "LoboDynamic/LoboDynamic.h"
#include "AutoDiff/AutoDiffCore.h"

#include "Functions/computeSVDMatrix3d.h"

typedef Eigen::Triplet<double> EIGEN_TRI;

Lobo::HyperelasticModel::HyperelasticModel(Lobo::LoboDynamicScene *scene_,
                                           Lobo::LoboTetMesh *tetmesh_)
    : scene(scene_), tetmesh(tetmesh_)
{
    elastic_material = NULL;
    is_sparse_sovler = true;
    isinvertible = true;
    useMCSFD = true;
    inversion_Threshold = 0.5;
    num_DOFs = tetmesh_->getNumVertex() * 3;

    colMajorMatrixToTeran[0] = 0;
    colMajorMatrixToTeran[1] = 4;
    colMajorMatrixToTeran[2] = 6;
    colMajorMatrixToTeran[3] = 3;
    colMajorMatrixToTeran[4] = 1;
    colMajorMatrixToTeran[5] = 8;
    colMajorMatrixToTeran[6] = 5;
    colMajorMatrixToTeran[7] = 7;
    colMajorMatrixToTeran[8] = 2;

    for (int i = 0; i < 9; i++)
        teranToColMajorMatrix[colMajorMatrixToTeran[i]] = i;
}

Lobo::HyperelasticModel::~HyperelasticModel()
{
    delete elastic_material;

    int numElements = tetmesh->getNumElements();

    for (int i = 0; i < numElements; i++)
    {
        delete internalforce_list[i];
        delete stiffness_list[i];
    }
}

void Lobo::HyperelasticModel::runXMLscript(pugi::xml_node &xml_node)
{
    Lobo::DynamicModel::runXMLscript(xml_node);

    this->tetmesh->precomputeElementData();

    if (xml_node.child("Material"))
    {
        pugi::xml_node material_node = xml_node.child("Material");
        double YoungsModulus = 100.0;
        double PossionRatio = 0.4;
        double Density = 1.0;
        if (material_node.child("YoungsModulus"))
        {
            YoungsModulus =
                material_node.child("YoungsModulus").attribute("value").as_double();
        }
        
        if (material_node.child("PossionRatio"))
        {
            PossionRatio =
                material_node.child("PossionRatio").attribute("value").as_double();
        }
        
        if (material_node.child("Density"))
        {
            Density = material_node.child("Density").attribute("value").as_double();
        }

        tetmesh->setAllMaterial(Density, YoungsModulus, PossionRatio);

        if (strcmp("StVK",
                   material_node.attribute("type").as_string()) ==
            0)
        {
            this->elastic_material =
                new TypeStVKMaterial<double>(tetmesh, 0, 500.0);
            materialtype = "StVK";
        }
        if (strcmp("NeoHookean",
                   material_node.attribute("type").as_string()) ==
            0)
        {
            this->elastic_material =
                new TypeNeoHookeanMaterial<double>(tetmesh, 0, 500.0);
            materialtype = "NeoHookean";
        }
    }

    this->isinvertible = false;

    if (xml_node.attribute("invertible"))
    {
        this->isinvertible = xml_node.attribute("invertible").as_bool();
    }

    if (xml_node.attribute("useMCSFD"))
    {
        this->useMCSFD = xml_node.attribute("useMCSFD").as_bool();
    }
}

void Lobo::HyperelasticModel::precompute()
{
    this->tetmesh->precomputeElementData(); // will check if already
                                            // precomptued inside the function

    // Dm Dm_inverse per element
    // volume per element

    // precompute sparse matrix topology
    precomputedFdU();
    initMultiThreadBuffer();
}

void Lobo::HyperelasticModel::computeEnergySparse(
    Eigen::VectorXd *free_variables, double *energy, Eigen::VectorXd *jacobi,
    Eigen::SparseMatrix<double> *hessian, int computationflags)
{
    currentdisplacement = *free_variables;
    if (computationflags & Computeflags_reset)
    {
        if (computationflags & Computeflags_energy)
            *energy = 0;

        if (computationflags & Computeflags_fisrt)
            jacobi->setZero();

        if (computationflags & Computeflags_second)
        {
            for (int i = 0; i < hessian->outerSize(); ++i)
                for (Eigen::SparseMatrix<double>::InnerIterator it(*hessian, i);
                     it; ++it)
                {
                    it.valueRef() = 0;
                }
        }
    }

    int num_ele = tetmesh->getNumElements();

#pragma omp parallel
    {
#pragma omp for
        for (int i = 0; i < num_ele; i++)
        {
            if (useMCSFD)
            {
                if (computationflags & (Computeflags_energy | Computeflags_fisrt |
                                        Computeflags_second))
                {
                    this->getTetForceMatrixCSFD(i, internalforce_list[i],
                                                stiffness_list[i]);
                }
                else if (computationflags &
                         (Computeflags_energy | Computeflags_fisrt))
                {
                    this->getTetForceCSFD(i, internalforce_list[i]);
                }
            }
            else
            {
                //test
                this->getTetForceMatrixAnalytical(i, internalforce_list[i],
                                                  stiffness_list[i], computationflags);

                // if (i == 0)
                // {
                //     std::cout << "========GT=========" << std::endl;
                //     std::cout << *internalforce_list[i] << std::endl;
                //     this->getTetForceCSFD(i, internalforce_list[i]);
                //     std::cout << "========CSFD=========" << std::endl;
                //     std::cout << *internalforce_list[i] << std::endl;
                // }
            }
        }
    }

    for (int i = 0; i < num_ele; i++)
    {
        assignTetElementForceAndMatrix(i, energy, jacobi, hessian,
                                       computationflags, 1.0);
    }
}

void Lobo::HyperelasticModel::computeStiffnessMatrixTopology(
    Eigen::SparseMatrix<double> *K)
{
    K->resize(num_DOFs, num_DOFs);
    int numVertices = tetmesh->getNumVertex();
    int numElements = tetmesh->getNumElements();

    // tetmesh 4 nodes
    int numElementVertices = tetmesh->numElementVertices;
    std::vector<int> vertices(numElementVertices);
    std::vector<EIGEN_TRI> entrys;

    for (int ele = 0; ele < numElements; ele++)
    {
        for (int ver = 0; ver < numElementVertices; ver++)
        {
            vertices[ver] =
                tetmesh->tet_indices[ele * numElementVertices + ver];
        }

        for (int i = 0; i < numElementVertices; i++)
        {
            for (int j = 0; j < numElementVertices; j++)
            {
                for (int k = 0; k < 3; k++)
                    for (int l = 0; l < 3; l++)
                    {
                        // non-fixed
                        entrys.push_back(EIGEN_TRI(3 * vertices[i] + k,
                                                   3 * vertices[j] + l, 1.0));
                    }
            }
        }
    }

    K->setFromTriplets(entrys.begin(), entrys.end());
}

void Lobo::HyperelasticModel::precomputedFdU()
{
    int numElement = tetmesh->getNumElements();

    std::vector<Eigen::Matrix3d> dF_tet(12);

    for (int i = 0; i < numElement; i++)
    {
        this->diffDeformationGradient(i, dF_tet);
        dF.push_back(dF_tet);

        Eigen::MatrixXd dF_du_tet(9, 12);
        Eigen::MatrixXd dF_du_tet9X9(9, 9);

        for (size_t j = 0; j < 12; j++)
        {
            for (size_t i = 0; i < 9; i++)
            {
                dF_du_tet(i, j) = dF_tet[j].data()[i];
            }
        }
        for (size_t j = 0; j < 9; j++)
        {
            for (size_t i = 0; i < 9; i++)
            {
                dF_du_tet9X9(i, j) = dF_tet[j].data()[i];
            }
        }

        dF_du.push_back(dF_du_tet);
        dF_du_9X9.push_back(dF_du_tet9X9);
    }
}

void Lobo::HyperelasticModel::initMultiThreadBuffer()
{
    int numElements = tetmesh->getNumElements();
    internalforce_list.resize(numElements);
    stiffness_list.resize(numElements);
    for (int i = 0; i < numElements; i++)
    {
        internalforce_list[i] = new Eigen::VectorXd(12);
        stiffness_list[i] = new Eigen::MatrixXd(12, 12);
    }
    energy_list.resize(numElements);
}

void Lobo::HyperelasticModel::computeElementDeformationshapeMatrix(
    int eleid, Eigen::Matrix3d &Ds)
{
    Ds.setZero();
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            Ds.data()[j * 3 + i] =
                currentdisplacement[tetmesh->tet_indices[eleid * 4 + j] * 3 +
                                    i] -
                currentdisplacement[tetmesh->tet_indices[eleid * 4 + 3] * 3 +
                                    i];
        }
    }
    Ds += tetmesh->getTetElement(eleid)->Dm;
}

void Lobo::HyperelasticModel::diffDeformationGradient(
    int elementid, std::vector<Eigen::Matrix3d> &dF)
{
    Eigen::Matrix3d dDs[12];
    Eigen::Matrix3d dm_inverse;
    dm_inverse = tetmesh->getTetElement(elementid)->Dm_inverse;

    for (size_t i = 0; i < 9; i++)
    {
        dDs[i].setZero();
        dDs[i](i % 3, i / 3) = 1;
        dF[i] = dDs[i] * dm_inverse;
    }

    for (size_t i = 0; i < 3; i++)
    {
        dDs[i + 9].setZero();
        dDs[i + 9](i, 0) = -1;
        dDs[i + 9](i, 1) = -1;
        dDs[i + 9](i, 2) = -1;
        dF[i + 9] = dDs[i + 9] * dm_inverse;
    }
}

void Lobo::HyperelasticModel::getTetForceMatrixCSFD(
    int eleid, Eigen::VectorXd *internalforce, Eigen::MatrixXd *stiffness)
{
    Eigen::Matrix3d F, Ds, m_U, m_V, m_singularF, P;
    TetElementData *te = tetmesh->getTetElement(eleid);

    Eigen::Matrix3d Dm_inverse = te->Dm_inverse;
    computeElementDeformationshapeMatrix(eleid, Ds);
    F = Ds * Dm_inverse;

    if (isinvertible)
    {
        computeSVD(F, m_U, m_V, m_singularF, 1e-9, 1);

        for (int i = 0; i < 3; i++)
        {
            if (m_singularF.data()[i * 3 + i] < inversion_Threshold)
            {
                m_singularF.data()[i * 3 + i] = inversion_Threshold;
            }
        }
        F = m_U * m_singularF * m_V.transpose();
    }

    elastic_material->computeAutoDiffEnergyVectorMatrix(
        eleid, internalforce->data(), stiffness, F, energy_list[eleid]);

    energy_list[eleid] *= te->volume;
    for (int i = 0; i < 12; i++)
    {
        internalforce->data()[i] *= te->volume;
    }
    (*stiffness) *= te->volume;
}

void Lobo::HyperelasticModel::getTetForceCSFD(int eleid,
                                              Eigen::VectorXd *internalforce)
{
    Eigen::Matrix3d F, Ds, m_U, m_V, m_singularF, P;
    TetElementData *te = tetmesh->getTetElement(eleid);

    Eigen::Matrix3d Dm_inverse = te->Dm_inverse;
    computeElementDeformationshapeMatrix(eleid, Ds);
    F = Ds * Dm_inverse;
    if (isinvertible)
    {
        computeSVD(F, m_U, m_V, m_singularF, 1e-9, 1);
        for (int i = 0; i < 3; i++)
        {
            if (m_singularF.data()[i * 3 + i] < inversion_Threshold)
            {
                m_singularF.data()[i * 3 + i] = inversion_Threshold;
            }
        }
        F = m_U * m_singularF * m_V.transpose();
    }

    elastic_material->computeAutoDiffEnergyVector(eleid, internalforce->data(),
                                                  F, energy_list[eleid]);
    energy_list[eleid] *= te->volume;
    for (int i = 0; i < 12; i++)
    {
        internalforce->data()[i] *= te->volume;
    }
}

void Lobo::HyperelasticModel::getTetForceMatrixAnalytical(int eleid, Eigen::VectorXd *internalforce, Eigen::MatrixXd *stiffness, int computationflags)
{
    Eigen::Matrix3d F, Ds, m_U, m_V, m_singularF, P;
    Eigen::Matrix3d dP_dF[9];
    Eigen::MatrixXd m_12x9_dfdF(12, 9);
    TetElementData *te = tetmesh->getTetElement(eleid);
    Eigen::Matrix3d Dm_inverse = te->Dm_inverse;
    computeElementDeformationshapeMatrix(eleid, Ds);
    F = Ds * Dm_inverse;
    computeSVD(F, m_U, m_V, m_singularF, 1e-9, 1);
    if (isinvertible)
    {
        for (int i = 0; i < 3; i++)
        {
            if (m_singularF.data()[i * 3 + i] < inversion_Threshold)
            {
                m_singularF.data()[i * 3 + i] = inversion_Threshold;
            }
        }
        F = m_U * m_singularF * m_V.transpose();
    }

    double fhat[3], phat[3];
    fhat[0] = m_singularF.data()[0];
    fhat[1] = m_singularF.data()[4];
    fhat[2] = m_singularF.data()[8];

    ComputeDiagonalPFromStretches(eleid, fhat, phat, energy_list[eleid]);
    energy_list[eleid] *= te->volume;

    P.setZero();
    P.data()[0] = phat[0];
    P.data()[4] = phat[1];
    P.data()[8] = phat[2];

    //compute real P
    P = m_U * P * m_V.transpose();

    if (computationflags & Computeflags_fisrt)
    {
        for (int i = 0; i < 12; i++)
        {
            internalforce->data()[i] = 0;
            for (int j = 0; j < 9; j++)
            {
                internalforce->data()[i] += P.data()[j] * dF_du[eleid].data()[j + i * 9];
            }
            internalforce->data()[i] *= te->volume;
        }
    }

    if (computationflags & Computeflags_second)
    {
        compute_dPdF(eleid, dP_dF, fhat, m_U, m_V);
        compute_dfdF(eleid, dP_dF, m_12x9_dfdF, dF[eleid]);
        *stiffness = m_12x9_dfdF * dF_du[eleid] * te->volume;
    }
}

void Lobo::HyperelasticModel::assignTetElementForceAndMatrix(
    int eleid, double *energy, Eigen::VectorXd *internalforce,
    Eigen::SparseMatrix<double> *sparseMatrix, int flags, double weights)
{
    if (flags & Computeflags_energy)
    {
        *energy += this->energy_list[eleid];
    }

    if (flags & Computeflags_fisrt)
    {
        for (int i = 0; i < 4; i++)
        {
            int n = tetmesh->tet_indices[eleid * 4 + i];
            internalforce->data()[n * 3] +=
                internalforce_list[eleid]->data()[i * 3 + 0];
            internalforce->data()[n * 3 + 1] +=
                internalforce_list[eleid]->data()[i * 3 + 1];
            internalforce->data()[n * 3 + 2] +=
                internalforce_list[eleid]->data()[i * 3 + 2];
        }
    }

    if (flags & Computeflags_second)
    {
        int numElementVertices = tetmesh->numElementVertices;
        for (int c = 0; c < 4; c++)
        {
            for (int a = 0; a < 4; a++)
            {
                int row = tetmesh->tet_indices[eleid * 4 + c] * 3;
                int col = tetmesh->tet_indices[eleid * 4 + a] * 3;

                for (int r = 0; r < 3; r++)
                {
                    for (int l = 0; l < 3; l++)
                    {
                        int block_r = c * 3 + r;
                        int block_c = a * 3 + l;

                        int trurow = row + r;
                        int trucol = col + l;

                        int index =
                            column_[eleid]
                                   [3 * numElementVertices * block_c + block_r];
                        sparseMatrix->valuePtr()[index] +=
                            stiffness_list[eleid]
                                ->data()[block_c * 12 + block_r];
                    }
                }
            }
        }
    }
}

void Lobo::HyperelasticModel::ComputeDiagonalPFromStretches(int eleid, double *lambda, double *PDiag, double &energy)
{
    double invariants[3];

    double lambda2[3] = {lambda[0] * lambda[0], lambda[1] * lambda[1], lambda[2] * lambda[2]};
    double IC = lambda2[0] + lambda2[1] + lambda2[2];
    double IIC = lambda2[0] * lambda2[0] + lambda2[1] * lambda2[1] + lambda2[2] * lambda2[2];
    double IIIC = lambda2[0] * lambda2[1] * lambda2[2];

    invariants[0] = IC;
    invariants[1] = IIC;
    invariants[2] = IIIC;

    double dPsidI[3];

    energy = elastic_material->ComputeEnergy(eleid, invariants);

    elastic_material->ComputeEnergyGradient(eleid, invariants, dPsidI);

    Eigen::Matrix3d mat;

    mat.data()[0] = 2.0 * lambda[0];
    mat.data()[1] = 2.0 * lambda[1];
    mat.data()[2] = 2.0 * lambda[2];
    mat.data()[3] = 4.0 * lambda[0] * lambda[0] * lambda[0];
    mat.data()[4] = 4.0 * lambda[1] * lambda[1] * lambda[1];
    mat.data()[5] = 4.0 * lambda[2] * lambda[2] * lambda[2];
    mat.data()[6] = 2.0 * lambda[0] * lambda2[1] * lambda2[2];
    mat.data()[7] = 2.0 * lambda[1] * lambda2[0] * lambda2[2];
    mat.data()[8] = 2.0 * lambda[2] * lambda2[0] * lambda2[1];

    Eigen::Vector3d dPsidIV;
    dPsidIV.data()[0] = dPsidI[0];
    dPsidIV.data()[1] = dPsidI[1];
    dPsidIV.data()[2] = dPsidI[2];

    Eigen::Vector3d result = mat * dPsidIV;
    PDiag[0] = result.data()[0];
    PDiag[1] = result.data()[1];
    PDiag[2] = result.data()[2];
}

void Lobo::HyperelasticModel::compute_dPdF(int eleid, Eigen::Matrix3d dPdF[9], double *fhat, Eigen::Matrix3d &U, Eigen::Matrix3d &V)
{
    double sigma1square = fhat[0] * fhat[0];
    double sigma2square = fhat[1] * fhat[1];
    double sigma3square = fhat[2] * fhat[2];

    double invariants[3];
    invariants[0] = sigma1square + sigma2square + sigma3square;
    invariants[1] = (sigma1square * sigma1square +
                     sigma2square * sigma2square +
                     sigma3square * sigma3square);
    invariants[2] = sigma1square * sigma2square * sigma3square;

    double gradient[3];
    elastic_material->ComputeEnergyGradient(eleid, invariants, gradient);

    double hessian[6];
    elastic_material->ComputeEnergyHessian(eleid, invariants, hessian);

    double alpha11 = 2.0 * gradient[0] + 8.0 * sigma1square * gradient[1];
    double alpha22 = 2.0 * gradient[0] + 8.0 * sigma2square * gradient[1];
    double alpha33 = 2.0 * gradient[0] + 8.0 * sigma3square * gradient[1];
    double alpha12 = 2.0 * gradient[0] + 4.0 * (sigma1square + sigma2square) * gradient[1];
    double alpha13 = 2.0 * gradient[0] + 4.0 * (sigma1square + sigma3square) * gradient[1];
    double alpha23 = 2.0 * gradient[0] + 4.0 * (sigma2square + sigma3square) * gradient[1];

    double beta11 = 4.0 * sigma1square * gradient[1] - (2.0 * invariants[2] * gradient[2]) / sigma1square;
    double beta22 = 4.0 * sigma2square * gradient[1] - (2.0 * invariants[2] * gradient[2]) / sigma2square;
    double beta33 = 4.0 * sigma3square * gradient[1] - (2.0 * invariants[2] * gradient[2]) / sigma3square;
    double beta12 = 4.0 * fhat[0] * fhat[1] * gradient[1] - (2.0 * invariants[2] * gradient[2]) / (fhat[0] * fhat[1]);
    double beta13 = 4.0 * fhat[0] * fhat[2] * gradient[1] - (2.0 * invariants[2] * gradient[2]) / (fhat[0] * fhat[2]);
    double beta23 = 4.0 * fhat[1] * fhat[2] * gradient[1] - (2.0 * invariants[2] * gradient[2]) / (fhat[1] * fhat[2]);

    double gamma11 = gammaValue(0, 0, fhat, invariants, gradient, hessian);
    double gamma22 = gammaValue(1, 1, fhat, invariants, gradient, hessian);
    double gamma33 = gammaValue(2, 2, fhat, invariants, gradient, hessian);
    double gamma12 = gammaValue(0, 1, fhat, invariants, gradient, hessian);
    double gamma13 = gammaValue(0, 2, fhat, invariants, gradient, hessian);
    double gamma23 = gammaValue(1, 2, fhat, invariants, gradient, hessian);

    double x1111, x2222, x3333;
    double x2211, x3311, x3322;
    double x2121, x3131, x3232;
    double x2112, x3113, x3223;

    x1111 = alpha11 + beta11 + gamma11;
    x2222 = alpha22 + beta22 + gamma22;
    x3333 = alpha33 + beta33 + gamma33;

    x2211 = gamma12;
    x3311 = gamma13;
    x3322 = gamma23;

    x2121 = alpha12;
    x3131 = alpha13;
    x3232 = alpha23;

    x2112 = beta12;
    x3113 = beta13;
    x3223 = beta23;

    double dPdF_atFhat[81];
    memset(dPdF_atFhat, 0, sizeof(double) * 81);
    dPdF_atFhat[tensor9x9Index(0, 0, 0, 0)] = x1111;
    dPdF_atFhat[tensor9x9Index(0, 0, 1, 1)] = x2211;
    dPdF_atFhat[tensor9x9Index(0, 0, 2, 2)] = x3311;

    dPdF_atFhat[tensor9x9Index(1, 1, 0, 0)] = x2211;
    dPdF_atFhat[tensor9x9Index(1, 1, 1, 1)] = x2222;
    dPdF_atFhat[tensor9x9Index(1, 1, 2, 2)] = x3322;

    dPdF_atFhat[tensor9x9Index(2, 2, 0, 0)] = x3311;
    dPdF_atFhat[tensor9x9Index(2, 2, 1, 1)] = x3322;
    dPdF_atFhat[tensor9x9Index(2, 2, 2, 2)] = x3333;

    dPdF_atFhat[tensor9x9Index(0, 1, 0, 1)] = x2121;
    dPdF_atFhat[tensor9x9Index(0, 1, 1, 0)] = x2112;

    dPdF_atFhat[tensor9x9Index(1, 0, 0, 1)] = x2112;
    dPdF_atFhat[tensor9x9Index(1, 0, 1, 0)] = x2121;

    dPdF_atFhat[tensor9x9Index(0, 2, 0, 2)] = x3131;
    dPdF_atFhat[tensor9x9Index(0, 2, 2, 0)] = x3113;

    dPdF_atFhat[tensor9x9Index(2, 0, 0, 2)] = x3113;
    dPdF_atFhat[tensor9x9Index(2, 0, 2, 0)] = x3131;

    dPdF_atFhat[tensor9x9Index(1, 2, 1, 2)] = x3232;
    dPdF_atFhat[tensor9x9Index(1, 2, 2, 1)] = x3223;

    dPdF_atFhat[tensor9x9Index(2, 1, 1, 2)] = x3223;
    dPdF_atFhat[tensor9x9Index(2, 1, 2, 1)] = x3232;

    double eiejVector[9];
    memset(eiejVector, 0, sizeof(double) * 9);
    memset(dPdF, 0, sizeof(double) * 81);
    Eigen::Matrix3d elejmat;

    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {

            elejmat.setZero();
            elejmat.data()[j * 3 + i] = 1.0;

            Eigen::Matrix3d ut_eiej_v = U.transpose() * elejmat * V;
            double ut_eiej_v_TeranVector[9]; //in Teran order
            for (int k = 0; k < 9; k++)
                ut_eiej_v_TeranVector[colMajorMatrixToTeran[k]] = ut_eiej_v.data()[k];

            double dPdF_resultVector[9]; // not in teran order

            for (int innerRow = 0; innerRow < 9; innerRow++)
            {
                double tempResult = 0.0;
                for (int innerColumn = 0; innerColumn < 9; innerColumn++)
                {
                    tempResult += dPdF_atFhat[innerRow * 9 + innerColumn] * ut_eiej_v_TeranVector[innerColumn];
                }

                dPdF[j * 3 + i].data()[teranToColMajorMatrix[innerRow]] = tempResult;
            }
        }
    }

    for (size_t k = 0; k < 9; k++)
    {
        dPdF[k] = U * dPdF[k] * V.transpose();
    }
}

void Lobo::HyperelasticModel::compute_dfdF(int eleid, Eigen::Matrix3d (&dP_dF)[9], Eigen::MatrixXd &dfdF, std::vector<Eigen::Matrix3d> &dF)
{
    for (size_t i = 0; i < 9; i++)
    {
        for (size_t j = 0; j < 12; j++)
        {
            dfdF.data()[i * 12 + j] = (dF[j].transpose() * dP_dF[i]).trace();
        }
    }
}

double Lobo::HyperelasticModel::gammaValue(int i, int j, double sigma[3], double invariants[3], double gradient[3], double hessian[6])
{
    double tempGammaVec1[3];
    tempGammaVec1[0] = 2.0 * sigma[i];
    tempGammaVec1[1] = 4.0 * sigma[i] * sigma[i] * sigma[i];
    tempGammaVec1[2] = 2.0 * invariants[2] / sigma[i];
    double tempGammaVec2[3];
    tempGammaVec2[0] = 2.0 * sigma[j];
    tempGammaVec2[1] = 4.0 * sigma[j] * sigma[j] * sigma[j];
    tempGammaVec2[2] = 2.0 * invariants[2] / sigma[j];
    double productResult[3];
    productResult[0] = (tempGammaVec2[0] * hessian[0] + tempGammaVec2[1] * hessian[1] +
                        tempGammaVec2[2] * hessian[2]);
    productResult[1] = (tempGammaVec2[0] * hessian[1] + tempGammaVec2[1] * hessian[3] +
                        tempGammaVec2[2] * hessian[4]);
    productResult[2] = (tempGammaVec2[0] * hessian[2] + tempGammaVec2[1] * hessian[4] +
                        tempGammaVec2[2] * hessian[5]);
    return (tempGammaVec1[0] * productResult[0] + tempGammaVec1[1] * productResult[1] +
            tempGammaVec1[2] * productResult[2] + 4.0 * invariants[2] * gradient[2] / (sigma[i] * sigma[j]));
}
int Lobo::HyperelasticModel::tensor9x9Index(int i, int j, int m, int n)
{
    int rowIndex_in9x9Matrix = colMajorMatrixToTeran[3 * j + i];
    int columnIndex_in9x9Matrix = colMajorMatrixToTeran[3 * n + m];
    return (9 * rowIndex_in9x9Matrix + columnIndex_in9x9Matrix);
}