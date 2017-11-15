//
//  oneDsearch.cpp
//  pid
//
//  Created by Scott Clegg on 11/13/17.
//
// Class oneDsearch
// This class uses a one dimensional search with golden ratio to find the
// minimum error.
//

#include <iostream>
#include <math.h>
#include "oneDsearch.h"

using namespace std;


oneDsearch::oneDsearch(): isInitialized(false) {};

oneDsearch::~oneDsearch() {};

// Initialize the one D search algorithm

void oneDsearch::Init(double a, double b, double tolerance) {
    this->a = a;                    // Define initial parameter search bounds. Left bound is a.
    this->b = b;                    // Define initial parameter search bounds. Right bound is b.
    this->tolerance = tolerance;    // Search stopping tolerance
    step = GetErrorA;               // Set to initialize by getting error for parameter = a
    isInitialized = true;
}

// Return new parameter estimate
double oneDsearch::paramUpdate() {
    switch (step) {
        case GetErrorA:
            // Return a for evaluation
            return a;
            
        case GetErrorB:
            // Return b for evaluation
            return b;
            
        case GetErrorC:
            // Return parameter based on golden ratio of a and b
            c = b-(b-a)/psi;
            return c;
            
        case GetErrorD:
            // Return parameter based on golden ratio of a and b
            d = a+(b-a)/psi;
            return d;
            
        case AdjustParamC:
            // Return parameter based on golden ratio of a and b
            c = b+(a-b)/psi;
            return c;
            
        case AdjustParamD:
            // Return parameter based on golden ratio of a and b
            d = a+(b-a)/psi;
            return d;
            
        default:
            cout << "error check =  "<< step << " not defined " << endl;
            exit(0);
            break;
    }
    
}

// Method to get the magnitude of dp
bool oneDsearch::newError(double error) {
    switch (step) {
        case GetErrorA:
            // Set error for parameter a
            error_a = error;
            step = GetErrorB;
            return false;
            
        case GetErrorB:
            // Set error for parameter b
            error_b = error;
            step = GetErrorC;
            return false;
            
        case GetErrorC:
            // Set error for parameter c
            error_c = error;
            step = GetErrorD;
            return false;
            
        case GetErrorD:
            // Set error for parameter d
            error_d = error;
            
            // Now decide which way to adjust
            if(error_c < error_d) {
                b = d;
                error_b = error;
                d = c;
                error_d = error_c;
                step = AdjustParamC;
            } else {
                a = c;
                error_a = error;
                c = d;
                error_c = error_d;
                step = AdjustParamD;
            }
            return false;
            
        default:
            // Set new search bounds
            if(error_c < error) {
                b = d;
                error_b = error;
                d = c;
                error_d = error_c;
                step = AdjustParamC;
            } else {
                a = c;
                error_a = error;
                c = d;
                error_c = error_d;
                step = AdjustParamD;
            }
            // If search bounds are smaller than tolerance then done with search.
            if(fabs(a-b)<tolerance) {
                return true;
            } else {
                return false;
            }
    }
}


