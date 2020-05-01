#include "ActivationLayer.h"
#include <iostream>
#include "AutoDiff/LoboComplex.h"

LOBO_TEMPLATE_INSTANT_NN(ActivationLayer)

template<class TYPE>
ActivationLayer<TYPE>::ActivationLayer(const char* activation_type_, int n_)
{
	n = n_;
	this->n_input = n;
	this->n_output = n;
	leakyrelu_alpha = 0.2;

	if (strcmp(activation_type_, "tanh") == 0)
	{
		activation_type = ActivationLayer::tanh;
	}
	else if (strcmp(activation_type_, "relu") == 0)
	{
		activation_type = ActivationLayer::relu;
	}
	else if (strcmp(activation_type_, "sigmoid") == 0)
	{
		activation_type = ActivationLayer::sigmoid;
	}
	else if (strcmp(activation_type_, "linear") == 0)
	{
		activation_type = ActivationLayer::linear;
	}
	else if (strcmp(activation_type_, "LeakyReLU") == 0)
	{
		activation_type = ActivationLayer::leakyrelu;
	}

	std::cout << "activation => " << activation_type_ << std::endl;

}

template<class TYPE>
ActivationLayer<TYPE>::~ActivationLayer()
{
	
}

template<class TYPE>
void ActivationLayer<TYPE>::layerRun(TYPE* input, TYPE* output)
{
	switch (activation_type)
	{
	case ActivationLayer::tanh:
		for (int i = 0;i < n;i++)
		{
			output[i] = tanhFunction(input[i]);
		}
		break;
	case ActivationLayer::relu:
		for (int i = 0;i < n;i++)
		{
			output[i] = reluFunction(input[i]);
		}
		break;
	case ActivationLayer::sigmoid:
		for (int i = 0;i < n;i++)
		{
			output[i] = sigmoidFunction(input[i]);
		}
		break;
	case ActivationLayer::linear:
		for (int i = 0;i < n;i++)
		{
			output[i] = input[i];
		}
		break;
	case ActivationLayer::leakyrelu:
		for (int i = 0;i < n;i++)
		{
			output[i] = leakyreluFunction(input[i]);
		}
		break;
	default:
		break;
	}
	
}

template<class TYPE>
TYPE ActivationLayer<TYPE>::tanhFunction(TYPE input)
{
	//don't use this
	input = lobo::tanh(input);
	return input;
}

template<class TYPE>
TYPE ActivationLayer<TYPE>::reluFunction(TYPE input)
{
	if (lobo::largerReal(input, (double)0.0))
	{
		return input;
	}
	else
	{
		TYPE temp = (TYPE)0.0;
		return temp;
	}
}

template<class TYPE>
TYPE ActivationLayer<TYPE>::leakyreluFunction(TYPE input)
{
	if (lobo::smallerReal(input, (double)0.0))
	{
		return input*leakyrelu_alpha;
	}
	else
	{
		return input;
	}

}

template<class TYPE>
TYPE ActivationLayer<TYPE>::sigmoidFunction(TYPE input)
{
	TYPE tmp = (double)1.0 / ((double)1.0 + lobo::exp(-input));
	return tmp;
}



