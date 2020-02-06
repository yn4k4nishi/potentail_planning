//
// Created by ynakanishi on 2020/02/06.
//

#include <iostream>
#include <deque>
#include "object.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main(){

    std::cout << "--- start potential planning ---" << std::endl;

    StartPoint s(0,0);
    GoalPoint g(10,10);
    Obstacle o1(2,4);
    Obstacle o2(6,2);
    Obstacle o3(5,8);

    std::deque<ObjectBase*> objects;
    objects.push_back(&s);
    objects.push_back(&g);
    objects.push_back(&o1);
    objects.push_back(&o2);
    objects.push_back(&o3);

    //// plot
    std::vector<std::vector<double>> x, y, z;
    for (double i = -1; i <= 11;  i += 0.25) {
        std::vector<double> x_row, y_row, z_row;
        for (double j = -1; j <= 11; j += 0.25) {
            x_row.push_back(i);
            y_row.push_back(j);

            float tmp_z = 0.0f;
            for(auto &o : objects){
                tmp_z += o->potential(Eigen::Vector2f(i,j));
            }
            z_row.push_back(tmp_z);
        }
        x.push_back(x_row);
        y.push_back(y_row);
        z.push_back(z_row);
    }

    plt::plot_surface(x, y, z);
    plt::show();

    return 0;
}
