#include "BaseLayer.h"

LOBO_TEMPLATE_INSTANT_NN(BaseLayer);

template <class TYPE>
BaseLayer<TYPE>::BaseLayer()
{

}

template <class TYPE>
BaseLayer<TYPE>::~BaseLayer()
{
    free(output_vector);
}

template <class TYPE>
void BaseLayer<TYPE>::allocBuffer()
{
    output_vector = (TYPE*)malloc(sizeof(TYPE)*n_output);
}