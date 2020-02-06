//
// Created by ynakanishi on 2020/02/06.
//

#ifndef POTENTIAL_PLANNING_OBJECT_H
#define POTENTIAL_PLANNING_OBJECT_H

#include <Eigen/Core>

/*** Base Class ***/
class ObjectBase {
public:
    explicit ObjectBase(){}
    virtual ~ObjectBase() = default;

    // 斥力を返す
    virtual Eigen::Vector2f repulsive(const Eigen::Vector2f pos) = 0;

    // ポテンシャルを返す
    virtual float potential(const Eigen::Vector2f pos) = 0;

};

/*** Start Point ***/
class StartPoint : public ObjectBase {
private:
    Eigen::Vector2f pose;
    static constexpr float A = 10.0f;
public:
    StartPoint(float x, float y):pose(x,y){}
    ~StartPoint(){}

    // 斥力を返す
    Eigen::Vector2f repulsive(const Eigen::Vector2f pos){
        /*
         *  |F| = | A / r^2 |
         *  クーロン力の式と同様
         */
        return A / (this->pose - pos).squaredNorm() * (this->pose - pos).normalized();
    };

    // ポテンシャルを返す
    float potential(const Eigen::Vector2f pos){
        /*
         *  |F| = | A / r |
         *  クーロンポテンシャルの式と同様
         */
        return A / (this->pose - pos).norm();
    };
};

/*** Goal Point ***/
class GoalPoint : public ObjectBase {
private:
    Eigen::Vector2f pose;
    static constexpr float A = -10.0f;
public:
    GoalPoint(float x, float y):pose(x,y){}
    ~GoalPoint(){}

    // 斥力を返す
    Eigen::Vector2f repulsive(const Eigen::Vector2f pos){
        /*
         *  |F| = | A / r^2 |
         *  クーロン力の式と同様
         */
        return A / (this->pose - pos).squaredNorm() * (this->pose - pos).normalized();
    };

    // ポテンシャルを返す
    float potential(const Eigen::Vector2f pos){
        /*
         *  |F| = | A / r |
         *  クーロンポテンシャルの式と同様
         */
        return A / (this->pose - pos).norm();
    };
};

/*** Obstacle ***/
class Obstacle : public ObjectBase {
private:
    Eigen::Vector2f pose;
    static constexpr float A = 3.0f;
public:
    Obstacle(float x, float y):pose(x,y){}
    ~Obstacle(){}

    // 斥力を返す
    Eigen::Vector2f repulsive(const Eigen::Vector2f pos){
        /*
         *  |F| = | A / r^2 |
         *  クーロン力の式と同様
         */
        return A / (this->pose - pos).squaredNorm() * (this->pose - pos).normalized();
    };

    // ポテンシャルを返す
    float potential(const Eigen::Vector2f pos){
        /*
         *  |F| = | A / r |
         *  クーロンポテンシャルの式と同様
         */
        return A / (this->pose - pos).norm();
    };
};


#endif //POTENTIAL_PLANNING_OBJECT_H
