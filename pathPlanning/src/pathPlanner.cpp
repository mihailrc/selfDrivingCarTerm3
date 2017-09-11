//
// Created by Mihail Chirita on 9/9/17.
//
#include <cmath>
#include <vector>
#include <iostream>
#include "pathPlanner.h"
#include "traffic.h"

PathPlanner::PathPlanner(Waypoints waypoints): waypoints(waypoints) {

    time_step = 0.02;    // s
    number_of_points = 50; //we are going to calculate trajectory for the next second or 50 points
    // There is a portion of the track where speed gets close to 50 mph. This has to do with curvature of the road
    speed_limit = 46.5 * conversion_factor;   // convert to m/s.
    lane_width = 3.95;    // Lane width, Adjusted to compensate for simulator outside of lane error in lane 2.

    cleareance = 12.0;    //room needed in front and rear in order to change lanes i.e. there is no car within this distance
    this->waypoints = waypoints;
    this->traffic = Traffic();
}

void PathPlanner::process(double car_s,
                          double car_d,
                          double car_speed,
                          std::vector<double> previous_path_x,
                          std::vector<double> previous_path_y,
                          double end_s,
                          double end_d,
                          std::vector<Vehicle> tracked_vehicles) {
    // Update state
    this->car_s_ = car_s;
    this->car_d_ = car_d;
    this->car_speed_ = car_speed * conversion_factor;  // convert to m/s
    this->previous_path_x_ = previous_path_x;
    this->previous_path_y_ = previous_path_y;
    this->end_s_ = end_s;
    this->end_d_ = end_d;
    double search_radius = 50; //only take into account vehicles within the search radius

    current_lane = static_cast<int>(car_d_ / lane_width);
    if (!previous_path_x_.empty()) {
        current_lane = static_cast<int>(end_d_ / lane_width);
    }

    this->traffic = Traffic(tracked_vehicles, lane_width, speed_limit, search_radius, car_s, previous_path_x.size()*time_step);

    if (previous_path_x_.size() > 2) {
        int best_lane = find_best_lane();
        recalculate_path(best_lane);
    } else {
        recalculate_path(current_lane);
    }
}

/**
 * determines lane with lowest cost
 */
int PathPlanner::find_best_lane() {

    std::vector<double> costs = {calculateCost(current_lane, 0),
                                 calculateCost(current_lane, 1),
                                 calculateCost(current_lane, 2)};
    int best_lane = 0;
    double lowest_cost = costs[0];
    for (int i = 0; i < 3; i++) {
            if (costs[i] < lowest_cost) {
                lowest_cost = costs[i];
                best_lane = i;
            }
        }
    return best_lane;
}

/** uses traffic information to calculate cost of switching to another lane
 or staying in same lane **/
double PathPlanner::calculateCost(int currentLane, int targetLane) {
    int high_cost = 100000;
    double totalCost = 0;
    //add penalty for changing lane
    totalCost += abs(currentLane - targetLane) * 10;
    //add a high cost for leaving the road
    if (targetLane < 0 || targetLane > 2) {
        totalCost += high_cost;
    }
    //add high penalty if cannot change lane
    if (currentLane != targetLane && !can_change_lane(targetLane)) {
        totalCost += high_cost;
    }

    //add high cost if we want to cross 2 lanes but the middle lane cannot be crossed
    if (((currentLane == 0 && targetLane == 2) || (currentLane == 2 && targetLane == 0)) && !can_change_lane(1)) {
        totalCost += high_cost;
    }
    //add penalty for slow speeds
    totalCost += 100 * (1.0 - traffic.get_lane_speed(targetLane) / speed_limit);
    //add penalty if vehicle in front
    totalCost += 100 * (1.0 - traffic.distance_to_nearest_vehicle_ahead(targetLane) / 30.0);
    return totalCost;
}

bool PathPlanner::can_change_lane(int target_lane){
    return traffic.distance_to_nearest_vehicle_ahead(target_lane)>=cleareance &&
           traffic.distance_to_nearest_vehicle_behind((target_lane)) >= cleareance;
}


void PathPlanner::recalculate_path(int target_lane) {

    clean_up_processed_points();

    tk::spline spline_d = calculate_d_spline(target_lane);

    add_new_points(target_lane, spline_d);

}

void PathPlanner::clean_up_processed_points() {

    //reset x and y
    next_x_ = {};
    next_y_ = {};

    //keep the points that were not processed
    for (int i = 0; i < previous_path_x_.size(); i++) {
        next_x_.push_back(previous_path_x_[i]);
        next_y_.push_back(previous_path_y_[i]);
    }

    // remove points that were processed in the previous step
    while (next_s_.size() > previous_path_x_.size()) {
        next_s_.erase(next_s_.begin());
        next_d_.erase(next_d_.begin());
    }
}

tk::spline PathPlanner::calculate_d_spline(int target_lane) const {

    tk::spline spline_d;

    double target_d = (target_lane + 0.5) * lane_width;
    double smoothing_distance = 30;
    //need more smoothing when switching two lanes
    if(abs(current_lane - target_lane) == 2){
        smoothing_distance *= 2;
    }

    //use previously calculated points if they exist. Otherwise use latest car position.
    if (next_s_.size() >= 2) {
        spline_d.set_points(
                {next_s_[next_s_.size() - 2], next_s_.back(), next_s_.back() + smoothing_distance, next_s_.back() + smoothing_distance * 2},
                {next_d_[next_d_.size() - 2], next_d_.back(), target_d, target_d});
    } else {
        spline_d.set_points(
                {car_s_ - 0.5, car_s_, car_s_ + smoothing_distance, car_s_ + smoothing_distance * 2},
                {car_d_, car_d_, target_d, target_d});
    }
    return spline_d;
}

void PathPlanner::add_new_points(int target_lane, const tk::spline &spline_d) {
    double s;
    double speed;

    //use latest values otherwise use sensor fusion data
    if (next_s_.size() >= 2) {
        s = next_s_.back();
        speed = (next_s_.back() - next_s_[next_s_.size() - 2]) / time_step;
    } else {
        s = car_s_;
        speed = car_speed_;
    }

    double speed_adjustment = calculate_speed_adjustment(target_lane, speed);

    std::vector<double> xy;
    double d;

    while (next_x_.size() < number_of_points) {
        speed += speed_adjustment;
        s += time_step * speed;
        d = spline_d(s);
        xy = waypoints.getXY(s, d);

        // append new points
        next_x_.push_back(xy[0]);
        next_y_.push_back(xy[1]);
        next_s_.push_back(s);
        next_d_.push_back(d);
    }
}

//calculates how much to adjust speed at every step.
double PathPlanner::calculate_speed_adjustment(int target_lane, double speed) {
    int number_of_new_points = (number_of_points - previous_path_x_.size());

    auto target_speed = std::min(speed_limit, traffic.get_lane_speed(target_lane));

    double speed_diff  = (target_speed - speed);

    //adjust speed in small increments or decrements
    double speed_diff_per_step = 0;
    if(speed_diff<0) {
      speed_diff_per_step = std::max(-0.15, speed_diff / number_of_new_points);
    }else {
        speed_diff_per_step = std::min(0.15, speed_diff / number_of_new_points);
    }
    return speed_diff_per_step;
}



