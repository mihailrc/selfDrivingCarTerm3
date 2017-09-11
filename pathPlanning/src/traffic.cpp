//
// Created by Mihail Chirita on 9/10/17.
//

#include <vector>
#include "vehicle.h"
#include "traffic.h"

Traffic::Traffic(std::vector<Vehicle> vehicles, double lane_width, double speed_limit, double search_radius,
                 double reference_vehicle_position, double time_delay) {
    this->vehicles = vehicles;
    this->lane_width = lane_width;
    this->speed_limit = speed_limit;
    this->search_radius = search_radius;
    this->reference_vehicle_position = reference_vehicle_position;
    this->time_delay = time_delay;

    double big_value = search_radius * 1000;

    nearest_ahead_ = {big_value, big_value, big_value};
    nearest_behind = {big_value, big_value, big_value};
    lane_speeds_ = {speed_limit, speed_limit, speed_limit};

    for (auto &vehicle : vehicles) {
        auto vehicle_lane = static_cast<int>(vehicle.d / lane_width);

        double tracked_pos = vehicle.newPosition(time_delay);

        if (std::abs(tracked_pos - reference_vehicle_position) <= search_radius) {
            if (tracked_pos > reference_vehicle_position) {
                if (vehicle.v < lane_speeds_[vehicle_lane]) {
                    lane_speeds_[vehicle_lane] = vehicle.v;
                }
                if (tracked_pos - reference_vehicle_position <= nearest_ahead_[vehicle_lane]) {
                    nearest_ahead_[vehicle_lane] = tracked_pos - reference_vehicle_position;
                }
            } else {
                if (reference_vehicle_position - tracked_pos < nearest_behind[vehicle_lane]) {
                    nearest_behind[vehicle_lane] = reference_vehicle_position - tracked_pos;
                }
            }
        }
    }
}

double Traffic::get_lane_speed(int lane_number) {
    return lane_speeds_[lane_number];
}

double Traffic::distance_to_nearest_vehicle_ahead(int lane_number) {
    return nearest_ahead_[lane_number];
}

double Traffic::distance_to_nearest_vehicle_behind(int lane_number) {
    return nearest_behind[lane_number];
}

