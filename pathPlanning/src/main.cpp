#include <uWS/uWS.h>
#include <iostream>
#include <vector>
#include "json.hpp"
#include "vehicle.h"
#include "pathPlanner.h"
#include "waypoints.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}


int main() {
    uWS::Hub h;

    // Initialize waypoints
    Waypoints waypoints = Waypoints("../data/highway_map.csv");

    // Initialize path planner
    PathPlanner planner = PathPlanner(waypoints);

    h.onMessage([&planner, &waypoints](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto telemetry_data = json::parse(s);

                string event = telemetry_data[0].get<string>();

                if (event == "telemetry") {
                    // telemetry_data[1] is the data JSON object

                    // Main car's localization Data
                    double car_x = telemetry_data[1]["x"];
                    double car_y = telemetry_data[1]["y"];
                    double car_s = telemetry_data[1]["s"];
                    double car_d = telemetry_data[1]["d"];
                    double car_yaw = telemetry_data[1]["yaw"];
                    double car_speed = telemetry_data[1]["speed"];

                    // Previous path data given to the Planner
                    auto previous_path_x = telemetry_data[1]["previous_path_x"];
                    auto previous_path_y = telemetry_data[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = telemetry_data[1]["end_path_s"];
                    double end_path_d = telemetry_data[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = telemetry_data[1]["sensor_fusion"];

                    json msgJson;

                    // Construct vectors of previous paths
                    vector<double> v_previous_path_x;
                    vector<double> v_previous_path_y;

                    if(previous_path_x.size() > 0){
                        for(int i=0; i < previous_path_x.size(); i++){
                            v_previous_path_x.push_back((double)(previous_path_x[i]));
                            v_previous_path_y.push_back((double)(previous_path_y[i]));
                        }
                    }

                    // Construct vector of tracked vehicle objects
                    vector<Vehicle> vehicles;
                    if(sensor_fusion.size() > 0){
                        for(int i=0; i < sensor_fusion.size(); i++){
                            auto vehicle = sensor_fusion[i];

                            Vehicle new_vehicle(vehicle[0],   // id
                                                vehicle[1],   // x
                                                vehicle[2],   // y
                                                vehicle[3],   // vx
                                                vehicle[4],   // vy
                                                vehicle[5],   // s
                                                vehicle[6]);  // d

                            vehicles.push_back(new_vehicle);
                        }
                    }

                    planner.process(car_s, car_d,
                                    car_speed,
                                    v_previous_path_x, v_previous_path_y,
                                    end_path_s, end_path_d,
                                    vehicles);

                    // Path is made up of 50 points that the car will visit sequentially every .02 seconds
                    msgJson["next_x"] = planner.next_x_;
                    msgJson["next_y"] = planner.next_y_;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
