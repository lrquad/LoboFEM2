#include "CUDABaseLayer.h"
#include <iostream>

Lobo::CUDABaseLayer::CUDABaseLayer()
{

}

Lobo::CUDABaseLayer::~CUDABaseLayer()
{
    free(cpu_input);
    free(cpu_output);
    //cudaFree(cuda_input);
    //cudaFree(cuda_output);
}

void Lobo::CUDABaseLayer::allocBuffer()
{
    std::cout<<"allocBuffer "<<n_input<<" " << n_output<<std::endl;
    cpu_input = (float*)malloc(sizeof(float)*n_input);
    cpu_output = (float*)malloc(sizeof(float)*n_output);

    //cudaMalloc((void**)&cuda_input, sizeof(float) * n_input);
   // cudaMalloc((void**)&cuda_output, sizeof(float) * n_output);

    input_r_r.resize(n_input);
    input_r_i.resize(n_input);
    input_i_r.resize(n_input);
    input_i_i.resize(n_input);

    output_r_r.resize(n_output);
    output_r_i.resize(n_output);
    output_i_r.resize(n_output);
    output_i_i.resize(n_output);

}