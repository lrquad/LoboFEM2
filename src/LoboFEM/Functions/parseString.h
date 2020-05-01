#pragma once
#include <vector>
#include <sstream>
#include <string>

namespace Lobo {

	template<typename T>
	void parseString(std::string input, std::vector<T> &output)
	{
		output.clear();
		std::stringstream iss(input);
		T value;
		while (iss >> value)
			output.push_back(value);
	}

	template<typename T>
	std::vector<T> parseString(std::string input)
	{
		std::vector<T> output;
		output.clear();
		std::stringstream iss(input);
		T value;
		while (iss >> value)
			output.push_back(value);
		return output;
	}

	template<typename T>
	int parseString(std::string input, T* &array_output)
	{
		if(array_output!=NULL)
		free(array_output);

		std::vector<T> output;
		output.clear();
		std::stringstream iss(input);
		T value;
		while (iss >> value)
			output.push_back(value);

		array_output = (T*)malloc( sizeof(T)*output.size() );
		memcpy(array_output, output.data(), sizeof(T)*output.size());
		return output.size();
	}

}