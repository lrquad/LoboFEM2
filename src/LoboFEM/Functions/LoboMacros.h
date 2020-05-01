#pragma once

#define LOBO_TEMPLATE_INSTANT(MyClass) \
    template class MyClass<double>;

#define LOBO_TEMPLATE_INSTANT_NN(MyClass) \
    template class MyClass<double>;       \
    template class MyClass<LoboComplexDualt>; \

#define TYPE_MAX 999999;

