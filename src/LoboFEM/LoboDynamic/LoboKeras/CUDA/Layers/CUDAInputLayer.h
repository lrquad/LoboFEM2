#pragma once
#include "CUDABaseLayer.h"

namespace Lobo
{
class CUDAInputLayer : public CUDABaseLayer
{
    virtual void layerRun(float *input, float *output){};

    virtual void layerRun(CUDABaseLayer *inputlayer){};

    virtual void layerRunValue(CUDABaseLayer* inputlayer){};

};
} // namespace Lobo