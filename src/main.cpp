//
// Created by ynakanishi on 2020/02/06.
//

#include <iostream>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

int main(){

    std::cout << "--- start potential planning ---" << std::endl;

    plt::plot({1,3,2,4});
    plt::show();

    return 0;
}
