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

    virtual float getDistance(const Eigen::Vector2f pos) = 0;

    virtual Eigen::Vector2f getPose() = 0;

};

/*** Start Point ***/
class StartPoint : public ObjectBase {
private:
    Eigen::Vector2f pose;
    static constexpr float A = 1000.0f;
public:
    StartPoint(float x, float y):pose(x,y){}
    ~StartPoint(){}

    // 斥力を返す
    Eigen::Vector2f repulsive(const Eigen::Vector2f pos){
        /*
         *  |F| = | A / r^2 |
         *  クーロン力の式と同様
         */
        return A / (this->pose - pos).squaredNorm() * (pos - this->pose).normalized();
    };

    // ポテンシャルを返す
    float potential(const Eigen::Vector2f pos){
        /*
         *  |F| = | A / r |
         *  クーロンポテンシャルの式と同様
         */
        return A / (pos - this->pose).norm();
    };

    float getDistance(const Eigen::Vector2f pos){
        return (this->pose - pos).norm();
    };

    Eigen::Vector2f getPose(){
        return this->pose;
    };
};

/*** Goal Point ***/
class GoalPoint : public ObjectBase {
private:
    Eigen::Vector2f pose;
    static constexpr float A = -5000.0f;
    static constexpr float R_0 = 1.0f;
public:
    GoalPoint(float x, float y):pose(x,y){}
    ~GoalPoint(){}

    // 斥力を返す
    Eigen::Vector2f repulsive(const Eigen::Vector2f pos){
        /*
         *  |F| = | A / (r- r_0)^2 |
         *  クーロン力の式と同様
         */
        return A / pow(this->getDistance(pos),2) * (pos - this->pose).normalized();
    };

    // ポテンシャルを返す
    float potential(const Eigen::Vector2f pos){
        /*
         *  |F| = | A / (r - r_0) |
         *  クーロンポテンシャルの式と同様
         */
        return A / this->getDistance(pos);
    };

    float getDistance(const Eigen::Vector2f pos){
        return (this->pose - pos).norm();
    };

    Eigen::Vector2f getPose(){
        return this->pose;
    };
};

/*** Obstacle ***/
class Obstacle : public ObjectBase {
private:
    Eigen::Vector2f pose;
    static constexpr float A = 3000.0f;
public:
    Obstacle(float x, float y):pose(x,y){}
    ~Obstacle(){}

    // 斥力を返す
    Eigen::Vector2f repulsive(const Eigen::Vector2f pos){
        /*
         *  |F| = | A / r^2 |
         *  クーロン力の式と同様
         */
        return A / (this->pose - pos).squaredNorm() * (pos - this->pose).normalized();
    };

    // ポテンシャルを返す
    float potential(const Eigen::Vector2f pos){
        /*
         *  |F| = | A / r |
         *  クーロンポテンシャルの式と同様
         */
        return A / (this->pose - pos).norm();
    };

    float getDistance(const Eigen::Vector2f pos){
        return (this->pose - pos).norm();
    };

    Eigen::Vector2f getPose(){
        return this->pose;
    };
};


#endif //POTENTIAL_PLANNING_OBJECT_H
