#ifndef PID_H
#define PID_H

class PID {
    // PID result lower and upper limits
    double lower_limit;
    double upper_limit;
    
public:
    /*
     * bool to indicate PID gains have been initialized
     */
    bool isInitialized;
    
    /*
     * Number of steps to take before error accumulation
     */
    int nSteps;
    
    /*
     * Number of PID calls
     */
    int nCalls;
    
    /*
     * PID gains
     */
    double *gains;
    
    /*
     * Controller output bounds
     */
    double *bounds;
    
    /*
     * Errors
     */
    double p_error;
    double i_error;
    double d_error;
    double error;
    
    /*
     * Coefficients
     */
    double Kp;
    double Ki;
    double Kd;
    
    /*
     * Constructor
     */
    PID();
    
    /*
     * Destructor.
     */
    virtual ~PID();
    
    /*
     * Initialize PID controller with
     * input parameters.
     */
    void Init(double *gains, double *bounds, int nStart);
    
    /*
     * Store the control output bounds
     */
    void StoreBounds(double *bounds);
    
    /*
     * Store the control output bounds
     */
    void StoreGains(double *gains);
    
    /*
     * Start PID controller.
     */
    void Start(double cte);
    
    /*
     * Update the PID error variables given cross track error.
     */
    void UpdateError(double cte);
    
    /*
     * Calculate the total PID error.
     */
    double TotalError();
    
    /*
     * Calculate control output
     */
    double ControlOutput(double cte);
};

#endif /* PID_H */
