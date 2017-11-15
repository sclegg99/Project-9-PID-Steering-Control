#include <uWS/uWS.h>
#include <vector>
#include <iostream>
#include <math.h>
#include "json.hpp"
#include "PID.h"
#include "Twiddle.h"
#include "oneDsearch.h"

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

struct Counters {
    int count = 0;
    double error = 0;
    double distance = 0;
};

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

    Counters counters;
    
    // Construct steering PID controller
    PID pidSteer;
    PID pidThrottle;
    
    int num_p = 3;
    int p_idx = 0;
    // Initial PID gains {Kp, Ki, Kd}
//    double steerGains[3] = {0.2666, 0.0042, 22.5840};
    double steerGains[3] = {0.2666, 0.0042, 22.5840};
    double past_gains[3];
    for(int j=0; j<num_p; j++)
        past_gains[j] = steerGains[j];

    double throttleGains[3] = {0.3439, 0.684585, 1.};
    
    // PID bounds
    double steerBounds[2] = {-1., 1.};
    double throttleBounds[2] = {0, 1.};
    
    // Initialize PID contoller
    pidSteer.StoreBounds(steerBounds);
    pidThrottle.StoreBounds(throttleBounds);
    
    pidSteer.StoreGains(steerGains);
    pidThrottle.StoreGains(throttleGains);
    
    // Construct One D Search
    oneDsearch od;
    double bounds[3][2] = {{.5, 2.}, {.001, .005}, {10., 30.}};
    
    h.onMessage([&od, &pidSteer, &bounds, &counters, &num_p, &p_idx, &past_gains](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(std::string(data).substr(0, length));
            if (s != "") {
                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();
                if (event == "telemetry") {
                    double cte = std::stod(j[1]["cte"].get<std::string>());
                    
                    // Need to gobble up data until reset has been achieved
                    if(!pidSteer.isInitialized && cte != 0.7598) {
//                        std::cout << "eating cte " << cte << std::endl;
                    } else {
                        // j[1] is the data JSON object
                        
                        double speed = std::stod(j[1]["speed"].get<std::string>());
//                        double angle = std::stod(j[1]["steering_angle"].get<std::string>());
                        double steerValue;
//                        double throttle = fmin(0.5, fmax(-1., (1. - 2.*fabs(cte))));
                        double throttle = 0.3;
                        
                        /*
                         * TODO: Calcuate steering value here, remember the steering value is
                         * [-1, 1].
                         * NOTE: Feel free to play around with the throttle and speed. Maybe use
                         * another PID controller to control the speed!
                         */
                        
                        // If new search then set pid gain to lhs value
                        if(!od.isInitialized) {
                            // initialize ond search with boundaries and tolerance
                            double a = bounds[p_idx][0];
                            double b = bounds[p_idx][1];
                            double tolerance = fabs(b-a)/100;
                            od.Init(a, b, tolerance);
                            
                            // set pid gain for search index p to a value
                            double *gains = pidSteer.gains;
                            pidSteer.gains[p_idx] = a;
                            pidSteer.StoreGains(gains);
                        }
                        
                        // Get PID control values given current cte and speed (or initalize if necessary)
                        if(pidSteer.isInitialized) {
                            steerValue = pidSteer.ControlOutput(cte);
//                            throttle = pidThrottle.ControlOutput(setSpeed - speed);
                        } else {
                            // Get gains based on which PID gains are being Twiddled
                            printf("Gain is %10.4f ",pidSteer.gains[p_idx]);
                            pidSteer.Init(cte);
//                            pidThrottle.Init(cte);
                        }
                        
                        // Send to simulator the new steering and throttle values
                        json msgJson;
                        msgJson["steering_angle"] = steerValue;
                        msgJson["throttle"] = throttle;
                        auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                        
                        // Accumulate the error and count the number of steps
                        counters.count++;
                        counters.error += cte*cte;
                        counters.distance += speed*.1/3600.;
//                        printf("Distance traveled %10.3f with current speed %6.2f and steering value %6.2f \n",counters.distance,speed,steerValue);
                        
                        // Check stopping criteria
                        if( (counters.count > 2000) || (fabs(cte) > 2.0) ) {
                            // Reset simulator
                            json msgJson;
                            msgJson["steering_angle"] = 0.;
                            msgJson["throttle"] = 0.;
                            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                            simulatorRestart(ws);
                            
                            // Normalize error by distance traveled
                            counters.error = sqrt(counters.error)/counters.distance;
                            
                            // Get current gains
                            double *gains = pidSteer.gains;
                            
                            // Push current error to one search routine
                            // if true is returned then search tolerance has been reached
                            bool searchDone = od.newError(counters.error);
                            
                            // Get new parameter estimate for one d search
                            double newGain = od.paramUpdate();
                            
                            // Push parameter to PID and store
                            pidSteer.gains[p_idx] = newGain;
                            pidSteer.StoreGains(gains);
                            
                            printf(" error %e \n",counters.error);
                            // set values and start over
                            pidSteer.isInitialized = false;
                            counters.count = 0;
                            counters.error = 0;
                            counters.distance = 0;
                            
                            if(searchDone) {
                                printf("Optimal gain for index %d is %10.4f\n",p_idx,pidSteer.gains[p_idx]);
                                od.isInitialized = false;
                                
                                // increment the p_idx
                                p_idx = (p_idx + 1) % num_p;

                                
                                // if p_idx has cycled back to the begining then check
                                // change of gain estimates
                                if(p_idx == 0) {
                                    double sum = 0;
                                    printf("Checking error ");
                                    for(int j=0; j<num_p; j++) {
                                        double diff = past_gains[j]-pidSteer.gains[j];
                                        printf(" %e ",diff);
                                        sum += diff*diff;
                                        past_gains[j] = pidSteer.gains[j];
                                    }
                                    printf(" and error is %e\n",sqrt(sum));
                                    if(sqrt(sum) < .001) {
                                        printf("*** Optimal Gains Are ***\n");
                                        for(int j=0; j<num_p; j++)
                                            printf("Gain[%d]=%10.4f\n",j,pidSteer.gains[j]);
                                        exit(0);
                                    }
                                }
                            }
                        }
                    }
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });
    
    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
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
        //        std::cout << "Connected!!!" << std::endl;
    });
    
    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });
    
    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
