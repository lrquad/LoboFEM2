#pragma once
#include "BaseLayer.h"

template<class TYPE> 
class DenseLayer:public BaseLayer<TYPE>
{
public:
	DenseLayer(TYPE* weights_,TYPE* bias_, int n,int m);
	~DenseLayer();

	virtual void layerRun(TYPE* input, TYPE* output);

protected:

	TYPE* weights;
	TYPE* bias;

	int n;
	int m;

};

