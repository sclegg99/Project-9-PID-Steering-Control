#include <uWS/uWS.h>
#include <iostream>
#include <math.h>
#include "json.hpp"
#include "PID.h"

// for convenience
using json = nlohmann::json;
using namespace std;

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
    auto b2 = s.find_last_of("]");
    if (found_null != string::npos) {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}



// Set reset message to the simulator
void simulatorRestart(uWS::WebSocket<uWS::SERVER> ws) {
    // send restart message to simulator
    string reset_msg = "42[\"reset\",{}]";
    ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

enum Optimize {steerOptimze, throttleOptimze, finishedOptimize};

int main()
{
    uWS::Hub h;

    // Construct steering PID controller
    PID pidSteer;
    PID pidThrottle;
    
    
    // Initial PID gains {Kp, Ki, Kd}
    double steerGains[3] = {0.2113, 0.0026, 21.5840};
    double throttleGains[3] = {0.1000, 0.0001, -0.0274};
    
    // PID bounds
    double steerBounds[2] = {-1., 1.};
    double throttleBounds[2] = {-1., 1.};
    
    // Initialize PID contoller
    pidSteer.StoreBounds(steerBounds);
    pidThrottle.StoreBounds(throttleBounds);
    
    pidSteer.StoreGains(steerGains);
    pidThrottle.StoreGains(throttleGains);
    
    
    // Desired set points
    double setCte = 0.;
    double setSpeed = 35.;
    
    // Number of steps before accumulating error
    int n2error = 0;
    
    // Initialize the PID controllers with inputs
    pidSteer.Init(steerGains, steerBounds, &setCte, &n2error);
    pidThrottle.Init(throttleGains, throttleBounds, &setSpeed, &n2error);
    
    double distance = 0.;
    double maxDistance = 10.;

    h.onMessage([&pidSteer, &pidThrottle, &distance, &maxDistance](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(string(data).substr(0, length));
            if (s != "") {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") {
                    double cte = stod(j[1]["cte"].get<string>());
                    // j[1] is the data JSON object
//                        const double Angle2Steer = -deg2rad(25.);
                    double speed = stod(j[1]["speed"].get<string>());
//                        double angle = stod(j[1]["steering_angle"].get<string>());
                    double throttleValue = 1.;
                    double steerValue = 0.;
                    
                    // Get PID control values given current cte and speed (or start controller if necessary)
                    if(pidSteer.isInitialized) {
                        steerValue = pidSteer.ControlOutput(cte);
                        throttleValue = pidThrottle.ControlOutput(speed);
                    } else {
                        pidSteer.Start(cte);
                        pidThrottle.Start(speed);
                    }
                    
                    // Send to simulator the new steering and throttle values
                    json msgJson;
                    msgJson["steering_angle"] = steerValue;
                    msgJson["throttle"] = throttleValue;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                    // Accumulate the error and count the number of steps
                    double distanceIncrement = speed*0.1/3600.;
                    distance += distanceIncrement; // assuming 0.1 sec per simulator increment
                    printf("CTE: %5.2f, Steering Value: %6.3f, Throttle: %6.3f, Distance Traveled: %6.2f\n",cte,steerValue, throttleValue, distance);
                    
                    // Check stopping criteria
                    if( distance > maxDistance) {
                        printf("Total steering error is %f\n",sqrt(pidSteer.GetError())/distance);
                        printf("Total speed error is %f\n",sqrt(pidThrottle.GetError())/distance);
                        simulatorRestart(ws);
                        exit(0);
                    }
                }
            } else {
                // Manual driving
                string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });
    
    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1)
        {
            res->end(s.data(), s.length());
        }
        else
        {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });
    
    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
//        cout << "Connected!!!" << endl;
    });
    
    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        cout << "Disconnected" << endl;
    });
    
    int port = 4567;
    if (h.listen(port))
    {
        cout << "Listening to port " << port << endl;
    }
    else
    {
        cerr << "Failed to listen to port" << endl;
        return -1;
    }
    h.run();
}
