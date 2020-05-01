#pragma once
#include "BaseLayer.h"
template<class TYPE>
class ActivationLayer :public BaseLayer<TYPE>
{
public:
	ActivationLayer(const char* activation_type_,int n);
	~ActivationLayer();

	typedef enum { tanh, relu,sigmoid,linear,leakyrelu} ActivationType;
	
	virtual void layerRun(TYPE* input, TYPE* output);

protected:

	TYPE tanhFunction(TYPE input);
	TYPE reluFunction(TYPE input);
	TYPE leakyreluFunction(TYPE input);
	TYPE sigmoidFunction(TYPE input);

	ActivationType activation_type;

	int n;
	TYPE leakyrelu_alpha;

};

