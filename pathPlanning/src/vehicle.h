//
// Created by Mihail Chirita on 9/9/17.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <cmath>

class Vehicle {
public:
    Vehicle(double id, double x, double y, double vx, double vy, double s, double d){
        this->id = id;
        this->x = x;
        this->y = y;
        this->vx = vx;
        this->vy = vy;
        this->s = s;
        this->d = d;
        this->v = std::sqrt(vx*vx + vy*vy);
    }

    ~Vehicle() = default;

    // Public Data
    double id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
    double v;

    //calculates new s value assuming constant speed
    double newPosition(double t){
        return s + v*t;
    }

};

#endif //PATH_PLANNING_VEHICLE_H
