#include "DynamicModel.h"

Lobo::DynamicModel::DynamicModel()
{
    num_DOFs = 0;
    is_sparse_sovler = true; // default sparse
    trigger = true; //turn on/off the model
    row_ = NULL;
    column_ = NULL;
}

Lobo::DynamicModel::~DynamicModel()
{

}

void Lobo::DynamicModel::runXMLscript(pugi::xml_node &xml_node)
{
    if(xml_node.attribute("trigger"))
    {
        trigger = xml_node.attribute("trigger").as_bool();
    }
}

void Lobo::DynamicModel::setAccelerationIndices(int **row, int **column)
{
    this->row_ = row;
    this->column_ = column;
}

void Lobo::DynamicModel::setAccelerationDiagIndices(int *diagonal)
{
    this->diagonal_ = diagonal;
}