//
//  oneDsearch.h
//  PID
//
//  Created by Scott Clegg on 11/13/17.
//

#ifndef oneDsearch_h
#define oneDsearch_h

enum Algorithm {GetErrorA, GetErrorB, GetErrorC, GetErrorD, AdjustParamC, AdjustParamD, CheckTolerance, Finish};

class oneDsearch {
    // golden ratio
    const double psi = 0.5*(1. + sqrt(5.));
    
    // stopping tolerance
    double tolerance;
    
    // parameter bounds
    double a;
    double b;
    
    // current parameter estimates
    double c;
    double d;
    
    // error estimates
    double error;
    double past_error;
    double error_a;
    double error_b;
    double error_c;
    double error_d;
    
    // flag to indicate which step of the one D search the algorithm is in
    Algorithm step;
    
public:
    
    // bool to indicate PID gains have been initialized
    bool isInitialized;
    
    /*
     * Constructor
     */
    oneDsearch();
    
    /*
     * Destructor.
     */
    virtual ~oneDsearch();
    
    /*
     * Initialize Twiddle parameters.
     */
    void Init(double a, double b, double tolerance);
    
    /*
     * push new error estimate to search algorithm
     */
    bool newError(double error);
    
    /*
     * return new parameter estimate
     */
    double paramUpdate();
    
    /*
     * Get the magnitude of an array
     */
    double Magnitued(double *dp);
};

#endif /* oneDsearch_h */
