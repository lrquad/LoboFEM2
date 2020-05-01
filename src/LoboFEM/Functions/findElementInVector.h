#pragma once
#include <vector>

template<typename T>
bool findElement(std::vector<T> pointer_list, T ele)
{
	return std::find(pointer_list.begin(), pointer_list.end(), ele) != pointer_list.end();
}

template<typename T>
int findElementIndex(std::vector<T> pointer_list, T ele)
{
	auto it = std::find(pointer_list.begin(), pointer_list.end(), ele);
	if (it == pointer_list.end())
	{
		return -1;
	}
	auto index = std::distance(pointer_list.begin(), it);
	return index;
}