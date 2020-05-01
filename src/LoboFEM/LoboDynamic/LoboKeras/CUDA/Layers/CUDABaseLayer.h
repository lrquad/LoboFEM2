#pragma once
#include <cuda.h>
#include <cuda_runtime.h>
#include <helper_cuda.h>
#include <Eigen/Dense>

#define CCE(x) checkCudaErrors(x)

namespace Lobo {

class CUDABaseLayer {
   public:
    CUDABaseLayer();
    ~CUDABaseLayer();


    virtual void layerRun(float* input, float* output) = 0;
    virtual void layerRun(CUDABaseLayer* inputlayer) = 0;
    virtual void layerRunValue(CUDABaseLayer* inputlayer) = 0;


    virtual void allocBuffer();

    int getInput() const { return n_input; }
    void setInput(int val) { n_input = val; }
    int getOutput() const { return n_output; }
    void setOutput(int val) { n_output = val; }
    
    Eigen::VectorXf input_r_r;
    Eigen::VectorXf input_r_i;
    Eigen::VectorXf input_i_r;
    Eigen::VectorXf input_i_i;
    
    Eigen::VectorXf output_r_r;
    Eigen::VectorXf output_r_i;
    Eigen::VectorXf output_i_r;
    Eigen::VectorXf output_i_i;

    float* cpu_input;
    float* cpu_output;

    float* cuda_input;
    float* cuda_output;

   protected:
    int n_input;
    int n_output;
};
}  // namespace Lobo
