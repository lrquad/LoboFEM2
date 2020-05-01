#include "LoboKerasModel.h"
#include "Functions/deleteSTDvector.h"
#include "Layers/DenseLayer.h"
#include "Layers/ActivationLayer.h"
#include <fstream>
#include <iostream>

LOBO_TEMPLATE_INSTANT_NN(LoboKerasModel)

template <class TYPE>
LoboKerasModel<TYPE>::LoboKerasModel()
{
	s_map_type["InputLayer"] = input_layer;
	s_map_type["Dense"] = dense_layer;
	s_map_type["LeakyReLU"] = leakyReLu_layer;
	data_min = 0;
	data_scale = 1.0;

}

template <class TYPE>
LoboKerasModel<TYPE>::~LoboKerasModel()
{
	deleteStdvectorPointer(NN_layers);
}

template <class TYPE>
void LoboKerasModel<TYPE>::loadNN(const char* filename)
{
	std::cout<<"load keras model " << filename<<std::endl;
	std::ifstream instream(filename);
	int number_of_layers;
	instream >> number_of_layers;
	int last_output = 0;
	for (int i = 0;i < number_of_layers;i++)
	{
		std::string layer_type_name;
		instream >> layer_type_name;
		std::cout << layer_type_name << std::endl;
		if (s_map_type[layer_type_name]==input_layer)
		{
			continue;
		}

		if (s_map_type[layer_type_name] == leakyReLu_layer)
		{
			addLayers(new ActivationLayer<TYPE>("LeakyReLU", last_output));
		}

		if (s_map_type[layer_type_name] == dense_layer)
		{
			std::string activation_type;
			instream >> activation_type;

			int num_weights;
			instream >> num_weights;

			int n, m;
			instream >> n >> m;
			TYPE* weights = (TYPE*)malloc(sizeof(TYPE)*m*n);
			memset(weights, 0, sizeof(TYPE)*m*n);
			
			for (int i = 0;i < n;i++)
			{
				for (int j = 0;j < m;j++)
				{
					double tmp;
					instream >> tmp;
					weights[j*n + i] = tmp;
				}
			}


			instream >> m;
			TYPE* bias = (TYPE*)malloc(sizeof(TYPE)*m);
			memset(bias, 0, sizeof(TYPE)*m);

			for (int i = 0;i < m;i++)
			{
				double tmp;
				instream >> tmp;
				bias[i] = tmp;
			}

			addLayers(new DenseLayer<TYPE>(weights,bias,n,m));
			//add activation
			addLayers(new ActivationLayer<TYPE>(activation_type.c_str(),m));
			free(weights);
			free(bias);
			last_output = m;
		}
	}

	instream.close();

	n_input = NN_layers[0]->getInput();
	n_output = NN_layers[NN_layers.size() - 1]->getOutput();
}

template <class TYPE>
void LoboKerasModel<TYPE>::addLayers(BaseLayer<TYPE>* layer)
{
	NN_layers.push_back(layer);
	layer->allocBuffer();
}

template <class TYPE>
void LoboKerasModel<TYPE>::predict(TYPE* input, TYPE* output)
{
	TYPE* hidden_vecotr = (TYPE*)malloc(sizeof(TYPE)*n_input);
	memset(hidden_vecotr, 0, sizeof(TYPE)*n_input);
	for (int i = 0;i < n_input;i++)
	{
		hidden_vecotr[i] = input[i];
	}

	for (int i = 0;i < NN_layers.size();i++)
	{
		int num_output = NN_layers[i]->getOutput();
		//TYPE* output_vector = (TYPE*)malloc(sizeof(TYPE)*num_output);
		memset(NN_layers[i]->output_vector, 0, sizeof(TYPE)*num_output);

		NN_layers[i]->layerRun(hidden_vecotr, NN_layers[i]->output_vector);

		free(hidden_vecotr);
		hidden_vecotr = (TYPE*)malloc(sizeof(TYPE)*num_output);
		memcpy(hidden_vecotr, NN_layers[i]->output_vector, sizeof(TYPE)*num_output);
		//free(output_vector);
	}

	for (int i = 0;i < n_output;i++)
	{
		output[i] = hidden_vecotr[i]*data_scale+data_min;
	}
	free(hidden_vecotr);
}

