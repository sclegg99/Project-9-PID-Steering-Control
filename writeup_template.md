# PID Control Project

The goal of this project is to implement a PID controller for steering control and then to optimize the controller gains. The optimized controller is tested using the UDACITY simulator ![alt text][image1] and completion of a circuit of the simulator track meets the project specifications (see [Rubric Points ](https://review.udacity.com/#!/rubrics/824/view)).

## Implementation

The simulator provides the cross track error (cte) which is feed into the PID controller.  The car steering control is output by the PID controller.  The PID controller was coded as a C++ class based on the skeleton provided for this project.

The PID gains were optimized using a gradient decent method, referred to as Twiddle. A detailed explanation of [Twiddle](https://www.youtube.com/watch?v=2uQ2BSzDvXs)  is available elsewhere.  I chose to use an error function that was the mean squared cte summed over all the simulation steps as illustrated in the following equation.

![alt text][image2]

The Error started accumulation after 100 simulation steps.  Then the Error was accumulated until the car had traveled approximately 1.5 miles (assuming a simulator time step of 100ms).  Finally the error was normalized by the distance (d) traveled after the start of the Error accumulation.

One of the issues with the Twiddle routine is that does not save the past Error history.  Therefore, it frequently makes Error evaluations for parameter estimates that have either already been evaluated or beyond the range of parameter estimates that have been previously evaluated.  Hence the routine can be quite inefficient due to the unnecessary Error evaluations.

An alternate to the Twiddle routine which performed a one dimensional search along each parameter axis using a golden ratio step size.  The one dimensional search method was not very efficient either.  I would speculate that the slope of the cost function (Error) was small, hence, many parameter evaluation were required to reach the Error minimum.

In addition to using PID control for the car steering, PID control of the speed was also implement.  In the case of speed control, a preset speed was choses (variable setSpeed) and the difference between the actual speed and setSpeed was used for feedback to the PID controller.  The output of the speed controller was the car throttle setting.

## Reflection

The Kp gain has the effect of causing the car to steer in the opposite direction of the cte.  Figure 2 illustrates this effect with Kp set to 0.1, 0.21, and 0.5 while Kd and Ki were set to zero.  Regardless of the magnitude of the Kp value, the car would oscillate from one side of the track to the other side until the car would eventually leave the road.  The figure also shows that the rate of correction increased with larger Kp values. It is desirable to have "quick" steering correction, hence a large value of Kp is needed.

![alt text][image3]

The steering oscillations can be damped with an appropriate value of Kd. Figure 3 illustrates the effect of varying Kd on the steering control. This figure plots the cte as a function of distance for Kp=0.21 and Ki=0.  It can be observed from this plot that the steering oscillations have been significantly dampened. The figure also shows that there is a consistent tracking bias (the cte is always positive) with values of Kd=10 and 20.

 ![alt text][image4]

 The tracking bias is dealt with by introducing Ki.  Figure 4 shows the effect of Ki with Kp=.21 and Kd=20.  This figure clearly shows that the car is not tracking about the center of the track.  Although it still shows some tendency to oscillate around cte=0.

 ![alt text][image5]

 For the final submission, a setSpeed of 35 miles per hour was selected.  The "optimal gains", as found by the Twiddle routine, for the steering control and speed control are listed below.

 | Gain  | Steering      | Throttle      |
 |:-----:|:-------------:|:-------------:|
 | Kp    |  0.2123       |  0.1000       |
 | Ki    |  0.0026       |  0.0000       |
 | Kd    | 21.5840       | -0.0274       |

Alternate values of setSpeed were evaluate (such as 45mph). However, optimal values of gain for the steering control were never identified because the car would always run off the track as it entered the corners.  The root cause seems to lie in the fact that the car does has no way to "anticipate" that it is about to enter a curve and hence prepare for entering the curve by either slowing down or applying a larger steering correction.  A solution to this might be to combine the PID steering control with a line finding algorithm (ala CARND project 3 on Behavioral Cloning).

[//]: # (Image References)

[image1]: ./figures/Simulator_Shot.png "Simulator Illustration"
[image2]: ./figures/Error_Equation.png "Error Equation"
[image3]: ./figures/VariationofKp.png "Effect of Kp"
[image4]: ./figures/VariationofKd.png "Effect of Kd"
[image5]: ./figures/VariationofKi.png "Effect of Ki"
