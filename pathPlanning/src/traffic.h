//
// Created by Mihail Chirita on 9/10/17.
//

#ifndef PATH_PLANNING_TRAFFIC_H
#define PATH_PLANNING_TRAFFIC_H

#include <vector>
#include "vehicle.h"

class Traffic{

private:
    std::vector<double> nearest_ahead_;
    std::vector<double> nearest_behind;
    std::vector<double> lane_speeds_;

public:

    std::vector<Vehicle> vehicles;
    double lane_width;
    double speed_limit;
    double search_radius;
    double reference_vehicle_position;
    double time_delay;

    // Constructor
    Traffic(){};
    Traffic(std::vector<Vehicle> vehicles, double lane_width, double speed_limit, double search_radius,
    double reference_vehicle_position, double time_delay);

    // Destructor
    ~Traffic()= default;

    double get_lane_speed(int lane_number);
    double distance_to_nearest_vehicle_ahead(int lane_number);
    double distance_to_nearest_vehicle_behind(int lane_number);


};

#endif //PATH_PLANNING_TRAFFIC_H
