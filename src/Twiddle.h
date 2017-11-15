//
//  Twiddle.h
//  PID
//
//  Created by Scott Clegg on 11/6/17.
//

#ifndef Twiddle_h
#define Twiddle_h

#include "PID.h"

enum Step {Initialize, CheckDp, NextIndex, Forward, Backward, Done};

class Twiddle {
    // stopping tolerance
    double tolerance;
    
    // magnitude of dp
    double magDp;
    
    // flag to indicate which step of the Twiddle check the routine is in
    Step check;
    
public:
    // PID controller class
    PID pid;
    
    // bool to indicate PID gains have been initialized
    bool isInitialized;
    
    // number of parameters
    int p_num;
    
    // parameter p and search step dp
    double *p;
    double *dp;
    
    // Twiddle counter
    int count;
    
    // Twiddle distance
    double distance;
    
    // Twiddle maximum distance
    double maxDistance;
    
    // steps before error evaluation
    int numSteps;
    
    // Current error
    double error;
    
    // Twiddle error
    double best_error;
    
    // index of parameter to change
    int p_idx;
    
    /*
     * Constructor
     */
    Twiddle();
    
    /*
     * Destructor.
     */
    virtual ~Twiddle();
    
    /*
     * Initialize Twiddle parameters.
     */
    void Init(double *p, double *dp, int p_num, double tolerance);
    
    /*
     * Check if number of steps has been exceeded
     */
    bool NumStepCheck();
    
    /*
     * Update the parameters using Twiddle algorithm
     */
    bool Update();

    /*
     * Get the magnitude of an array
     */
    double Magnitued(double *dp);
    
    /*
     * Set Twiddle error
     */
    void SetError(double error, int minSteps, int actualSteps);
};

#endif /* Twiddle_h */
