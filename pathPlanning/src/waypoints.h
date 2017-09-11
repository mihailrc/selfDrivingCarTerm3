//
// Created by Mihail Chirita on 9/9/17.
//

#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H

#include <fstream>
#include <vector>
#include "spline.h"

class Waypoints {
public:

    // read waypoints from file
    Waypoints(std::string map_file);

    ~Waypoints() = default;

    // Find closest waypoint given x,y position. It can be in front or behind the car
    int ClosestWaypoint(double x, double y);

    // Find next waypoint. This can only be in front of the car
    int NextWaypoint(double x, double y, double theta);

    // Transform from Cartesian coordinates to Frenet coordinates
    std::vector<double> getFrenet(double x, double y, double theta);

    // Transform from Frenet coordinates Cartesian coordinates using splines
    std::vector<double> getXY(double s, double d);

    std::vector<double> waypoints_x;
    std::vector<double> waypoints_y;
    std::vector<double> waypoints_s;
    std::vector<double> waypoints_dx;
    std::vector<double> waypoints_dy;

    double track_length;

private:
    tk::spline spline_x;
    tk::spline spline_y;
    tk::spline spline_dx;
    tk::spline spline_dy;
};


#endif //PATH_PLANNING_MAP_H