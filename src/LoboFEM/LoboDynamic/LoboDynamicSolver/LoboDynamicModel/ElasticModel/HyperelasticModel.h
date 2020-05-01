#pragma once
#include "LoboVolumtricMesh/LoboTetMesh.h"
#include "LoboDynamic/LoboDynamicScene.h"
#include "LoboDynamic/LoboDynamicSolver/LoboDynamicModel/DynamicModel.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

template <class TYPE>
class TypeIsotropicMaterial;

namespace Lobo
{

class HyperelasticModel : public DynamicModel
{
private:
    /* data */
public:
    HyperelasticModel(LoboDynamicScene *scene_, LoboTetMesh *tetmesh_);
    ~HyperelasticModel();

    virtual void precompute();
    virtual void computeStiffnessMatrixTopology(Eigen::SparseMatrix<double> *K);

    virtual void runXMLscript(pugi::xml_node &xml_node);

    virtual void computeEnergySparse(Eigen::VectorXd *free_variables, double *energy, Eigen::VectorXd *jacobi, Eigen::SparseMatrix<double> *hessian, int computationflags);
    virtual void computeEnergyDense(Eigen::VectorXd *free_variables, double *energy, Eigen::VectorXd *jacobi, Eigen::MatrixXd *hessian, int computationflags){};

    std::string materialtype;
    bool isinvertible;
    double inversion_Threshold;
    bool useMCSFD;

protected:
    virtual void precomputedFdU();
    virtual void initMultiThreadBuffer();

    virtual void computeElementDeformationshapeMatrix(int eleid, Eigen::Matrix3d &Ds);
    virtual void diffDeformationGradient(int elementid, std::vector<Eigen::Matrix3d> &dF);

    virtual void getTetForceMatrixCSFD(int eleid, Eigen::VectorXd *internalforce, Eigen::MatrixXd *stiffness);
    virtual void getTetForceCSFD(int eleid, Eigen::VectorXd *internalforce);

    virtual void getTetForceMatrixAnalytical(int eleid, Eigen::VectorXd *internalforce, Eigen::MatrixXd *stiffness, int computationflags);


	virtual void assignTetElementForceAndMatrix(int eleid,double* energy, Eigen::VectorXd *internalforce, Eigen::SparseMatrix<double>* sparseMatrix, int flags, double weights = 1.0);


    virtual void ComputeDiagonalPFromStretches(int eleid, double* lambda, double* PDiag, double &energy);
    virtual void compute_dPdF(int eleid, Eigen::Matrix3d dPdF[9], double* fhat, Eigen::Matrix3d &U, Eigen::Matrix3d &V);
	virtual void compute_dfdF(int eleid, Eigen::Matrix3d (&dP_dF)[9], Eigen::MatrixXd &dfdF, std::vector<Eigen::Matrix3d> &dF);


    inline double gammaValue(int i, int j, double sigma[3], double invariants[3], double gradient[3], double hessian[6]);
	int tensor9x9Index(int i, int j, int m, int n);

    LoboDynamicScene *scene;
    LoboTetMesh *tetmesh;
    TypeIsotropicMaterial<double> *elastic_material;

    Eigen::VectorXd currentdisplacement; //buffer

    //precompute
    std::vector<std::vector<Eigen::Matrix3d>> dF; // 12* numelements
    std::vector<Eigen::MatrixXd> dF_du;           // 9X12
    std::vector<Eigen::MatrixXd> dF_du_9X9;       // 9X12

    std::vector<Eigen::VectorXd *> internalforce_list;
    std::vector<Eigen::MatrixXd *> stiffness_list;
    std::vector<double> energy_list;

    //
    int colMajorMatrixToTeran[9];
	int teranToColMajorMatrix[9];

};
} // namespace Lobo