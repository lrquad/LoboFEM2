#include "ConstrainModel.h"
#include <fstream>
#include "LoboDynamic/LoboDynamic.h"

Lobo::ConstrainModel::ConstrainModel(LoboTetMesh *tetmesh_) : tetmesh(tetmesh_)
{
    num_constraints = 0;
    constrained_DoFs = NULL;
    num_DOFs = tetmesh->getNumVertex() * 3;
    weight_stiffness = 1000.0; //default value;
}

Lobo::ConstrainModel::~ConstrainModel()
{
    free(constrained_DoFs);
}

void Lobo::ConstrainModel::loadConstraints(const char *filename)
{
    std::ifstream inputstream(filename);
    inputstream >> num_constraints;
    constrained_DoFs = (int *)malloc(sizeof(int) * num_constraints);
    for (int i = 0; i < num_constraints; i++)
    {
        inputstream >> constrained_DoFs[i];
    }
}

void Lobo::ConstrainModel::runXMLscript(pugi::xml_node &xml_node)
{
    Lobo::DynamicModel::runXMLscript(xml_node);

    if(xml_node.child("ConstraintsFile"))
    {
        std::string filepath = xml_node.child("ConstraintsFile").text().as_string();
        this->loadConstraints(Lobo::getPath(filepath.c_str()).c_str());
    }

    if(xml_node.attribute("weight"))
    {
        weight_stiffness = xml_node.attribute("weight").as_double();
    }

}

void Lobo::ConstrainModel::computeEnergySparse(Eigen::VectorXd *free_variables, double *energy, Eigen::VectorXd *jacobi, Eigen::SparseMatrix<double> *hessian, int computationflags)
{
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

    //add constrain energy
    if (computationflags & Computeflags_energy)
    {
        for (int i = 0; i < num_constraints; i++)
        {
            int index = constrained_DoFs[i];
            (*energy)+=free_variables->data()[index]*free_variables->data()[index]*weight_stiffness;
        }
    }

    if (computationflags & Computeflags_fisrt)
    {
        for (int i = 0; i < num_constraints; i++)
        {
            int index = constrained_DoFs[i];
            jacobi->data()[index]+=2.0*free_variables->data()[index]*weight_stiffness;
        }
    }

    if (computationflags & Computeflags_second)
    {
        for(int i=0;i<num_constraints;i++)
        {
            int index = constrained_DoFs[i];
            hessian->valuePtr()[diagonal_[index]]+=2.0*weight_stiffness;
        }
    }
}