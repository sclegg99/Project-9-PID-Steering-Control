//
//  Twiddle.cpp
//  pid
//
//  Created by Scott Clegg on 11/6/17.
//
// Class Twiddle
// This class is used to optimize the parameter list p to a given tolerance
//
#include <iostream>
#include <math.h>
#include "Twiddle.h"

using namespace std;

/*
 * TODO: Complete the PID class.
 */

Twiddle::Twiddle(): check(Initialize), count(0), distance(0.), error(0.) {};

Twiddle::~Twiddle() {};

// Initialize the Twiddle search routine

void Twiddle::Init(double *p, double *dp, int p_num, double tolerance) {
    this->pid = pid;     // Define PID
    this->p = p;         // Initial parameter guess
    this->dp = dp;       // Initial search steps for each parameter
    
    this->tolerance = tolerance; // Stopping tolerance
    
    this->p_num = p_num;      // Number of parameters to adjust
    
    p_idx = 0;              // Initialize parameter to adjust index
}

// Check if number of parameter steps have been completed
bool Twiddle::NumStepCheck() {
    if(count > numSteps)
        return true;
    return false;
}

// Update the parameter estimates
// Returns true if tolerance is met...
bool Twiddle::Update() {
    switch (check) {
        case Initialize:
            // First time to call Twiddle...set best_error
            best_error = error;
            
            // Set check to StartIndex (ie p_idx=0)
            check = CheckDp;
            return false;

        case CheckDp:
            // Check if ||dp|| meets stopping criteria (ie < tolerance)
            magDp = Magnitued(dp);
            std::cout << "p_idx " << p_idx << " ||dp|| " << magDp << std::endl;
            
            // If ||dp|| is small then Twiddle is done
            if( magDp < tolerance) {
                check = Done;
                return true;
            }
            
            // Initialize p_idx to zero and set StartIndex
            p_idx = 0;
            
            // Increment p(0) = p(0) + dp(0)
            p[p_idx] += dp[p_idx];
            
            // set next check to Forward and get new simulator error
            check = Forward;
            return false;
            
        case NextIndex:
            // increment p index and cycle it back to 0 if p_num is reached
            p_idx = (p_idx+1) % p_num;
            if(p_idx == 0) {
                check = CheckDp;
                return false;
            }
            
            // increment p(i) by dp(i) and get new error
            p[p_idx] += dp[p_idx];
            
            // set the flag forward since this is a forward check (ie + increment of dp(i))
            check = Forward;
            
            // run new simulation
            return false;
            
        case Forward:
            // error an improvement
            if(error < best_error) {
                // Update best_error
                best_error = error;
                
                // Increase dp(i) step for next interation
                dp[p_idx] *= 1.1;
                
                check = NextIndex;
                
                // run new simulation
                return false;
            } else {
                // reduce p(i) by 2*dp(i)
                p[p_idx] -= 2*dp[p_idx];
                
                // set flag for backward since search direction has been reversed
                check = Backward;
                
                // run new simulation
                return false;
            }
            
        case Backward:
            // reverse search direction error evaluation
            if(error < best_error) {
                // if error improved update best_error and change dp(i) step size by 1.1
                best_error = error;
                dp[p_idx] *= 1.1;
            } else {
                // increment p(i) by dp(i) and then reduce the dp(i) step size
                p[p_idx] += dp[p_idx];
                dp[p_idx] *= 0.9;
            }
            
            // set flag for next index and run new simulation
            check = NextIndex;
            return false;
            
        default:
            cout << "error check =  "<< check << " not defined " << endl;
            exit(0);
            break;
    }

}

// Method to get the magnitude of dp
double Twiddle::Magnitued(double *dp) {
    double sum = 0;
    for(int i=0; i<p_num; i++)
        sum += dp[i]*dp[i];
    return sqrt(sum);
}

// Set Twiddle error based on accumulated error
// min steps and actual steps.
void Twiddle::SetError(double inError, int minSteps, int actualSteps) {
    if(actualSteps <= minSteps) {
        error = 1.e9;
    } else {
        // normalize the error by the number of steps since
        // the number of steps can vary
        error = inError/float(actualSteps-minSteps);
    }
}

