#include "CUDAKerasModel.h"
#include "Functions/deleteSTDvector.h"
#include "Layers/CUDABaseLayer.h"
#include "Layers/CUDADenseLayer.h"
#include "Layers/CUDAInputLayer.h"
#include <fstream>
#include <iostream>
#include <time.h>
Lobo::CUDAKerasModel::CUDAKerasModel()
{
    s_map_type["InputLayer"] = input_layer;
    s_map_type["Dense"] = dense_layer;
    s_map_type["LeakyReLU"] = leakyReLu_layer;
    data_min = 0;
    data_scale = 1.0;
}

Lobo::CUDAKerasModel::~CUDAKerasModel()
{
    deleteStdvectorPointer(NN_layers);
}

void Lobo::CUDAKerasModel::loadNN(const char *filename)
{
    std::cout<<"load NN " << filename << std::endl;

    std::ifstream instream(filename);
    int number_of_layers;
    instream >> number_of_layers;
    int last_output = 0;
    for (int i = 0; i < number_of_layers; i++)
    {
        std::string layer_type_name;
        instream >> layer_type_name;
        std::cout << layer_type_name << std::endl;
        if (s_map_type[layer_type_name] == input_layer)
        {
            addLayers(new CUDAInputLayer());
            continue;
        }

        if (s_map_type[layer_type_name] == dense_layer)
        {
            std::string activation_type;
            instream >> activation_type;

            int num_weights;
            instream >> num_weights;

            int n, m;
            instream >> n >> m;
            float *weights = (float *)malloc(sizeof(float) * m * n);
            memset(weights, 0, sizeof(float) * m * n);

            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < m; j++)
                {
                    double tmp;
                    instream >> tmp;
                    weights[i * m + j] = tmp;
                }
            }

            instream >> m;
            float *bias = (float *)malloc(sizeof(float) * m);
            memset(bias, 0, sizeof(float) * m);

            for (int i = 0; i < m; i++)
            {
                double tmp;
                instream >> tmp;
                bias[i] = tmp;
            }

            addLayers(new CUDADenseLayer(weights, bias, n, m,activation_type.c_str()));
            free(weights);
            free(bias);
            last_output = m;
        }
    }

    instream.close();
    NN_layers[0]->setInput(NN_layers[1]->getInput());
    NN_layers[0]->setOutput(NN_layers[1]->getInput());
    NN_layers[0]->allocBuffer();

    n_input = NN_layers[0]->getInput();
    n_output = NN_layers[NN_layers.size() - 1]->getOutput();
}

void Lobo::CUDAKerasModel::predict(double *input, double *output)
{
    for(int i=0;i<n_input;i++)
    {
        NN_layers[0]->output_r_r[i] = input[i];
    } 

    for(int i=1;i<NN_layers.size();i++)
    {
        NN_layers[i]->layerRunValue(NN_layers[i-1]);
    }

    for(int i=0;i<n_output;i++)
    {
        output[i] = NN_layers[NN_layers.size()-1]->output_r_r.data()[i]*data_scale+data_min;
    }
}

void Lobo::CUDAKerasModel::predict(double *input_r_r,double *input_r_i,double *input_i_r,double *input_i_i, double *output_r_r,double *output_r_i,double *output_i_r,double *output_i_i)
{
    for(int i=0;i<n_input;i++)
    {
        NN_layers[0]->output_r_r[i] = input_r_r[i];
        NN_layers[0]->output_r_i[i] = input_r_i[i];
        NN_layers[0]->output_i_r[i] = input_i_r[i];
        NN_layers[0]->output_i_i[i] = input_i_i[i];
    }    

    for(int i=1;i<NN_layers.size();i++)
    {
        NN_layers[i]->layerRun(NN_layers[i-1]);
        //std::cout<<NN_layers[i]->output_i_i.sum()<<std::endl;
    }

    for(int i=0;i<n_output;i++)
    {
        output_r_r[i] = NN_layers[NN_layers.size()-1]->output_r_r.data()[i]*data_scale+data_min;
        output_r_i[i] = NN_layers[NN_layers.size()-1]->output_r_i.data()[i]*data_scale;
        output_i_r[i] = NN_layers[NN_layers.size()-1]->output_i_r.data()[i]*data_scale;
        output_i_i[i] = NN_layers[NN_layers.size()-1]->output_i_i.data()[i]*data_scale;
    }   
}

void Lobo::CUDAKerasModel::addLayers(CUDABaseLayer *layer)
{
    NN_layers.push_back(layer);
	layer->allocBuffer();
}
