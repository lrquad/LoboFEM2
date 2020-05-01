#pragma once
#include "Functions/LoboMacros.h"
#include <vector>
#include <map>
#include <string>
#include "AutoDiff/AutoDiffCore.h"

template <class TYPE> class BaseLayer;

template <class TYPE>
class LoboKerasModel
{

public:
	LOBO_MAKE_TYPEDEFS(TYPE, t);

	LoboKerasModel();
	~LoboKerasModel();

	virtual void loadNN(const char* filename);

	virtual void addLayers(BaseLayer<TYPE>* layer);

	virtual void predict(TYPE* input, TYPE* output);


	enum layer_type_code {
		input_layer,
		dense_layer,
		leakyReLu_layer,
	};

	int getInput() const { return n_input; }
	void setInput(int val) { n_input = val; }
	int getOutput() const { return n_output; }
	void setOutput(int val) { n_output = val; }
	TYPE getData_min() const { return data_min; }
	void setData_min(TYPE val) { data_min = val; }
	TYPE getData_scale() const { return data_scale; }
	void setData_scale(TYPE val) { data_scale = val; }

protected:

	int n_input;
	int n_output;
	int n_hiddenlayers;

	std::vector<BaseLayer<TYPE>*> NN_layers;
	std::map<std::string, layer_type_code> s_map_type;

	//used in output
	TYPE data_min;
	TYPE data_scale;
};

