#include <iostream>
#include "PID.h"

#define getmax(a,b) ((a)>(b)?(a):(b))
#define getmin(a,b) ((a)<(b)?(a):(b))

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(): isInitialized(false) {};

PID::~PID() {};

// Initialize the PID controller
void PID::Init(double *gains,double *bounds,int n) {
    // First store the gains
    StoreGains(gains);
    
    // Now store the bounds of the control ouput
    StoreBounds(bounds);
    
    // Finall store the number of steps to take before
    // accumlating the error
    nSteps = n;
}

// Strt PID controller initializing the
// p, i and d error terms.
void PID::Start(double deviation) {
    // Initialize PID controller variables
    p_error = deviation;
    i_error = deviation;
    d_error = 0.;
    error = 0.;
    nCalls = 0;
    
    // Set isInitialized flag true
    isInitialized = true;
}


// Update of cte values
void PID::UpdateError(double deviation) {
    d_error = deviation - p_error;  // d(deviation)/dt for Kd compoenet
    p_error = deviation;            // current deviation for Kp component
    i_error += deviation;           // integral of deviation for Ki component
    nCalls++;
    if(nCalls > nSteps)
        error += deviation*deviation;
}

// Ouput the new steering angle base on
// the PID gains and cte history
double PID::TotalError() {
    double totalError = -(Kp*p_error + Ki*i_error + Kd*d_error);
    return totalError;
}

// Store the lower and upper limits of the
// allowable control output
void PID::StoreBounds(double *bounds) {
    lower_limit = bounds[0];
    upper_limit = bounds[1];
}

// Store the PID gains
void PID::StoreGains(double *gains) {
    this->gains = gains;
    Kp = gains[0];
    Ki = gains[1];
    Kd = gains[2];
}

// Function to return the new control value
// This is the result of first updating the error
// then calculateing the total error and finally
// checking that the new control value is within
// the allowable bounds
double PID::ControlOutput(double deviation) {
    UpdateError(deviation);
    double output = TotalError();
    output = getmax(output, lower_limit);
    output = getmin(output, upper_limit);
    return output;
}
