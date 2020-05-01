#pragma once
#include <vector>

template<typename T>
void deleteStdvectorPointer(std::vector<T> &pointer_list)
{
	for (typename std::vector<T>::iterator it = pointer_list.begin(); it != pointer_list.end(); ++it)
	{
		delete (*it);
		(*it) = NULL;
	}
	pointer_list.clear();
}

template<typename T>
void deleteStdvectorPointer(std::vector<T> *pointer_list)
{
	for (typename std::vector<T>::iterator it = pointer_list->begin(); it != pointer_list->end(); ++it)
	{
		delete (*it);
		(*it) = NULL;
	}
	delete pointer_list;
	pointer_list = NULL;
}