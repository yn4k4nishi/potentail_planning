//
// Created by ynakanishi on 2020/02/07.
//

#ifndef POTENTIAL_PLANNING_MACHINE_H
#define POTENTIAL_PLANNING_MACHINE_H


#include <Eigen/Core>

class Machine {
private:
    static constexpr float MASS = 10000; // g
    Eigen::Vector2f acc;
    Eigen::Vector2f vel;
    Eigen::Vector2f pose;
    float dt;
public:
    Machine(Eigen::Vector2f initial_pose, float dt)
        :dt(dt),pose(initial_pose),vel(0.0,0.0),acc(0.0,0.0){}
    ~Machine(){}

    Eigen::Vector2f getPose(){
        return this->pose;
    }

    void update(const Eigen::Vector2f force){
        updateAcc(force);
        updateVel();
        updatePos();
    }

private:
    void updateAcc(const Eigen::Vector2f force){
        this->acc = 1.0 / MASS * force;
    }
    void updateVel(){
        this->vel += dt * this->acc;
    }
    void updatePos(){
        this->pose += dt * this->vel;
    }

};


#endif //POTENTIAL_PLANNING_MACHINE_H
