#pragma once
#include "Functions/LoboMacros.h"
#include <vector>
#include <map>
#include <string>

namespace Lobo
{
class CUDABaseLayer;

class CUDAKerasModel
{
public:
    enum layer_type_code
    {
        input_layer,
        dense_layer,
        leakyReLu_layer,
    };

    CUDAKerasModel();
    ~CUDAKerasModel();

    virtual void loadNN(const char *filename);
    virtual void predict(double *input, double *output);
    virtual void predict(double *input_r_r,double *input_r_i,double *input_i_r,double *input_i_i, double *output_r_r,double *output_r_i,double *output_i_r,double *output_i_i);



    virtual void addLayers(CUDABaseLayer *layer);

    int getInput() const { return n_input; }
    void setInput(int val) { n_input = val; }
    int getOutput() const { return n_output; }
    void setOutput(int val) { n_output = val; }
    float getData_min() const { return data_min; }
    void setData_min(float val) { data_min = val; }
    float getData_scale() const { return data_scale; }
    void setData_scale(float val) { data_scale = val; }

protected:
    int n_input;
    int n_output;
    int n_hiddenlayers;

    float data_min;
    float data_scale;

    std::vector<CUDABaseLayer *> NN_layers;
    std::map<std::string, layer_type_code> s_map_type;
};
} // namespace Lobo