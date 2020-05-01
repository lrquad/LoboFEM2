#pragma once

#include "Functions/LoboMacros.h"
#include "AutoDiff/AutoDiffCore.h"

LOBO_MAKE_TYPEDEFS(double, t);

template <class TYPE>
class BaseLayer
{
public:
	

	BaseLayer();
	~BaseLayer();

	virtual void layerRun(TYPE* input, TYPE* output) = 0;
	virtual void allocBuffer();

	int getInput() const { return n_input; }
	void setInput(int val) { n_input = val; }
	int getOutput() const { return n_output; }
	void setOutput(int val) { n_output = val; }

	TYPE* output_vector;

protected:
	int n_input;
	int n_output;


};

