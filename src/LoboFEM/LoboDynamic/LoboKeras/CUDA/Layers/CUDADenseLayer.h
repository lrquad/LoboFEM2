#pragma once
#include "CUDABaseLayer.h"

namespace Lobo
{
    class CUDADenseLayer :public CUDABaseLayer
    {
        public:
        CUDADenseLayer(float* weights_,float* bias_,int n,int m,const char* activation_type_);
        ~CUDADenseLayer();

        typedef enum { tanh, relu,sigmoid,linear,leakyrelu} ActivationType;

        virtual void layerRun(float* input, float* output);

        virtual void layerRun(CUDABaseLayer* inputlayer);

        virtual void layerRunValue(CUDABaseLayer* inputlayer);

    protected:

        ActivationType activation_type;


        Eigen::MatrixXf weights;
        Eigen::VectorXf bias;

        float* cpu_weights;
        float* cpu_bias;
        float* gpu_weights;
        float* gpu_bias;

        int n;
        int m;

    };
}