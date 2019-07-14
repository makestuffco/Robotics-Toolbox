#include <iostream>
#include <memory>
#include "Mat.hpp"
using namespace std;

int main()
{
    std::vector<unique_ptr<double>> v;
    v.emplace_back(nullptr);
    return 0;
}