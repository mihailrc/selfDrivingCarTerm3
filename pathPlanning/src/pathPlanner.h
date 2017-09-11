//
// Created by Mihail Chirita on 9/9/17.
//
#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <vector>
#include "vehicle.h"
#include "waypoints.h"
#include "traffic.h"

class PathPlanner {
private:
    // Number of future points to generate
    int number_of_points;

    // Time steps between points
    double time_step;

    // Speed Limit
    double speed_limit;
    double lane_width;

    double cleareance;

    double conversion_factor = 0.44704; //to convert from mph to m/s

    Waypoints waypoints;

    Traffic traffic;

    void clean_up_processed_points();
    double calculateCost(int currentLane, int targetLane);

public:
    std::vector<double> next_x_;
    std::vector<double> next_y_;
    std::vector<double> next_s_;
    std::vector<double> next_d_;
    double car_s_;
    double car_d_;
    double car_speed_;
    std::vector<double> previous_path_x_;
    std::vector<double> previous_path_y_;
    double end_s_;
    double end_d_;
    int current_lane;

    // Constructor
    PathPlanner(Waypoints waypoints);

    // Destructor
    ~PathPlanner()= default;

    /*
     * Run planner on current state
     */
    void process(double car_s, double car_d, double car_speed,
                 std::vector<double> previous_path_x, std::vector<double> previous_path_y, double end_s, double end_d,
                 std::vector<Vehicle> tracked_vehicles);

    void recalculate_path(int target_lane);


    tk::spline calculate_d_spline(int target_lane) const;

    void add_new_points(int target_lane, const tk::spline &spline_d);

    double calculate_speed_adjustment(int target_lane, double speed);

    int find_best_lane();

    bool can_change_lane(int target_lane);
};



#endif //PATH_PLANNING_PLANNER_H