#include "CUDADenseLayer.h"
#include <cstring>
#include <iostream>
Lobo::CUDADenseLayer::CUDADenseLayer(float *weights_, float *bias_, int n_, int m_, const char *activation_type_)
{
    n = n_;
    m = m_;
    this->n_input = n_;
    this->n_output = m_;

    cpu_weights = (float *)malloc(sizeof(float) * n * m);
    memcpy(cpu_weights, weights_, sizeof(float) * n * m);

    cpu_bias = (float *)malloc(sizeof(float) * m);
    memcpy(cpu_bias, bias_, sizeof(float) * m);

    
    std::cout<<"weight matrix size: " <<m<<" " << n << std::endl;

    weights.resize(m, n);
    memcpy(weights.data(), weights_, sizeof(float) * n * m);
    bias.resize(m);
    memcpy(bias.data(), bias_, sizeof(float) * m);

    //copy to GPU
    CCE(cudaMalloc((void **)&gpu_weights, sizeof(float) * n * m));
    CCE(cudaMemcpy(gpu_weights, &cpu_weights, sizeof(float) * n * m, cudaMemcpyHostToDevice));
    CCE(cudaMalloc((void **)&gpu_bias, sizeof(float) * m));
    CCE(cudaMemcpy(gpu_bias, &cpu_bias, sizeof(float) * m, cudaMemcpyHostToDevice));

    if (strcmp(activation_type_, "tanh") == 0)
    {
        activation_type = ActivationType::tanh;
    }
    else if (strcmp(activation_type_, "relu") == 0)
    {
        activation_type = ActivationType::relu;
    }
    else if (strcmp(activation_type_, "sigmoid") == 0)
    {
        activation_type = ActivationType::sigmoid;
    }
    else if (strcmp(activation_type_, "linear") == 0)
    {
        activation_type = ActivationType::linear;
    }
    else if (strcmp(activation_type_, "LeakyReLU") == 0)
    {
        activation_type = ActivationType::leakyrelu;
    }

    std::cout << "activation => " << activation_type_ << std::endl;
}

Lobo::CUDADenseLayer::~CUDADenseLayer()
{
    free(cpu_weights);
    free(cpu_bias);
    cudaFree(gpu_weights);
    cudaFree(gpu_bias);
}

void Lobo::CUDADenseLayer::layerRun(float *input, float *output)
{
}

void Lobo::CUDADenseLayer::layerRun(Lobo::CUDABaseLayer *inputlayer)
{
    output_r_r = weights * inputlayer->output_r_r + bias;
    output_i_r = weights * inputlayer->output_i_r;
    output_r_i = weights * inputlayer->output_i_r;
    output_i_i = weights * inputlayer->output_i_i;

    //activation funtcion
    if (activation_type == ActivationType::relu)
    {
        for (int i = 0; i < m; i++)
        {
            if (output_r_r.data()[i] < 0)
            {
                output_r_r.data()[i] = 0;
                output_i_r.data()[i] = 0;
                output_r_i.data()[i] = 0;
                output_i_i.data()[i] = 0;
            }
        }
    }
    else if (activation_type == ActivationType::sigmoid)
    {
        for (int i = 0; i < m; i++)
        {
            float expx = std::exp(output_r_r[i]);
            float expnx = std::exp(-output_r_r[i]);
            float tmp = (1.0+expnx);
            output_r_r[i] = 1.0/tmp;
            float first = expnx/tmp/tmp;
            tmp = expx+1;
            float second = expx*(expx-1)/(tmp*tmp*tmp);
            output_i_i[i] = second*output_r_i[i]*output_i_r[i]+first*output_i_i[i];
            output_r_i[i] = first*output_r_i[i];
            output_i_r[i] = first*output_i_r[i];
        }
    }
}

void Lobo::CUDADenseLayer::layerRunValue(CUDABaseLayer* inputlayer)
{
    output_r_r = weights * inputlayer->output_r_r + bias;
    if (activation_type == ActivationType::relu)
    {
        for (int i = 0; i < m; i++)
        {
            if (output_r_r.data()[i] < 0)
            {
                output_r_r.data()[i] = 0;
            }
        }
    }
    else if (activation_type == ActivationType::sigmoid)
    {
        for (int i = 0; i < m; i++)
        {
            double expnx = std::exp(-output_r_r[i]);
            double tmp = (1.0+expnx);
            output_r_r[i] = 1.0/tmp;
        }
    }
}