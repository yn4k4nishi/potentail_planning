//
// Created by ynakanishi on 2020/02/06.
//

#include <iostream>
#include <deque>
#include "object.h"
#include "machine.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main(){

    std::cout << "--- start potential planning ---" << std::endl;

    StartPoint s(0,0);
    GoalPoint g(10,10);
    Obstacle o1(2,4);
    Obstacle o2(5,3);
    Obstacle o3(5,8);
    Obstacle o4(7,6);
    Obstacle o5(7,10);

    std::deque<ObjectBase*> objects;
    objects.push_back(&s);
    objects.push_back(&g);
    objects.push_back(&o1);
    objects.push_back(&o2);
    objects.push_back(&o3);
    objects.push_back(&o4);
    objects.push_back(&o5);

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

    Machine machine(Eigen::Vector2f(0.1,0.1),0.1);
    std::vector<float> x_plot,y_plot;
    int cnt = 0;
    while(g.getDistance(machine.getPose()) > 0.5){
//        std::cout << "loop ..." << std::endl;

        cnt ++;

        std::vector<float> x_obs,y_obs;
        Eigen::Vector2f force(0.0,0.0);
        for (auto &o : objects) {
            x_obs.push_back(o->getPose().x());
            y_obs.push_back(o->getPose().y());

            force += o->repulsive(machine.getPose());
        }
        machine.update(force);

        x_plot.push_back(machine.getPose().x());
        y_plot.push_back(machine.getPose().y());

//        if(cnt % 10 == 0) {
            plt::clf();
            plt::plot(x_plot, y_plot);
            plt::scatter(x_obs,y_obs);
            plt::xlim(-1, 11);
            plt::ylim(-1, 11);

            plt::title("potential planning figure");
            plt::pause(0.1);

//            cnt = 0;
//        }

    }

    return 0;
}
