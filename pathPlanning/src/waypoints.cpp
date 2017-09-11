//
// Created by Mihail Chirita on 9/9/17.
//
#include <fstream>
#include <cmath>
#include "json.hpp"
#include "waypoints.h"

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

Waypoints::Waypoints(std::string map_file) {
    std::ifstream in_map_(map_file.c_str(), std::ifstream::in);

    std::string line;
    while (getline(in_map_, line)) {
        std::istringstream iss(line);
        double x;
        double y;
        double s;
        double d_x;
        double d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        waypoints_x.push_back(x);
        waypoints_y.push_back(y);
        waypoints_s.push_back(s);
        waypoints_dx.push_back(d_x);
        waypoints_dy.push_back(d_y);
    }


    track_length = waypoints_s.back() + distance(waypoints_x[0], waypoints_y[0],
                                                 waypoints_x.back(), waypoints_y.back());


    //rollover some waypoints at the beginning of the track for smooth transition between laps
    for(int i=0;i<5;i++){
        waypoints_x.push_back(waypoints_x[i]);
        waypoints_y.push_back(waypoints_y[i]);
        waypoints_s.push_back(track_length  + waypoints_s[i]);
        waypoints_dx.push_back(waypoints_dx[i]);
        waypoints_dy.push_back(waypoints_dy[i]);

    }

    // Generate splines from s to x,y,dx and dy
    spline_x.set_points(waypoints_s, waypoints_x);
    spline_y.set_points(waypoints_s, waypoints_y);
    spline_dx.set_points(waypoints_s, waypoints_dx);
    spline_dy.set_points(waypoints_s, waypoints_dy);
}


//closest waypoint can be behind the car
int Waypoints::ClosestWaypoint(double x, double y)
{

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < waypoints_x.size(); i++)
    {
        double map_x = waypoints_x[i];
        double map_y = waypoints_y[i];
        double dist = distance(x, y, map_x, map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;
}


//this is the closest waypoint in front of the car
int Waypoints::NextWaypoint(double x, double y, double theta)
{

    int closestWaypoint = Waypoints::ClosestWaypoint(x,y);

    double map_x = waypoints_x[closestWaypoint];
    double map_y = waypoints_y[closestWaypoint];

    double heading = atan2( (map_y-y),(map_x-x) );

    double angle = std::abs(theta-heading);

    if(angle > M_PI/4)
    {
        closestWaypoint++;
    }

    return closestWaypoint;

}


// Transform from Cartesian coordinates to Frenet coordinates
std::vector<double> Waypoints::getFrenet(double x, double y, double theta)
{
    int next_wp = Waypoints::NextWaypoint(x, y, theta);

    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = waypoints_x.size()-1;
    }

    double n_x = waypoints_x[next_wp]-waypoints_x[prev_wp];
    double n_y = waypoints_y[next_wp]-waypoints_y[prev_wp];
    double x_x = x - waypoints_x[prev_wp];
    double x_y = y - waypoints_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000-waypoints_x[prev_wp];
    double center_y = 2000-waypoints_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(waypoints_x[i],waypoints_y[i],waypoints_x[i+1],waypoints_y[i+1]);
    }

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};

}

// Transform from Frenet coordinates Cartesian coordinates using splines
std::vector<double> Waypoints::getXY(double s, double d){
    //need to account for the fact that we rolled over waypoints at the beginning of the track
    while(s >= track_length){
        s -= track_length;
    }
    double x = spline_x(s) + d*spline_dx(s);
    double y = spline_y(s) + d*spline_dy(s);
    return {x,y};
}