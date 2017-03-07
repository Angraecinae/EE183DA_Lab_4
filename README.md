# EE183DA Lab 4 / 5

## Introduction:
In this lab report, we will simulate the movement of a two-wheeled robot from a starting point to a specified end point in {x, y, theta} given certain obstacles that would impede a straight line path to the endpoint, where {x, y} defines its location in the horizontal plane and {theta} defines its rotational state relative to the vertical z-axis. We will first derive a mathematical input/output model of the system dynamics, and define sensor and actuator responses based on experimental results and estimates from testing our sensors and components. Afterwards, we will use trajectory planning to map out a path (if such exists) that generates an obstacle free path to lead the robot from the starting point to the end point.

## Bill of Materials:
* MATLAB
* Inertial Measurement Unit (1x)
* Infrared Sensor and Infrared LED (2x)
* ESP8266 Microcontroller (1x)
* ESP12E Motor Shield (1x)
* FS90 Micro 9g Continuous Rotation Servo (2x)
* Jumper Wires (10x)

## Methods:
### Algorithm:

#### Kalman Filter Algorithm
  The main algorithm used in this system is the Kalman Filter. In simple terms, a Kalman filter is used to distinguish the actual signal from the noise, by making smart estimations given knowledge of the variance of the noise. To do so, the Kalman Filter simulates a measurement, given the input into the system, and then compares the simulated measurement to the actual measurement. Through this comparison, the Kalman filter generates a value, the Kalman gain, used with the covariance of the outputs, in order to make an optimal guess of the actual state evolution and correct itself to make better guesses in the future. In the case of our system, the Kalman filter takes in a state input: x, y, velocity in the x direction, velocity in the y direction, theta, and angular velocity , a covariance input and measurements from the sensors: velocity, measured by the rotation of the wheels, acceleration, theta, and angular rotation. The state and covariance is propagated through the state space equations, shown below in the mathematical formulation section, to obtain a simulated measurement, which is then compared against the measured velocities in the x, y direction as well as theta to generate a Kalman gain. The Kalman gain is fed back to the equation to estimate the next state, which is then reiterated through the same process against new measurements. See Figure 1 below for details:

Figure 1: Schematic of Kalman Filter design. The plant is represented by blocks A, B, T and H, which show a closed loop path. The top path is the representation of the actual measurement whereas the bottom path is the simulated measurement. “Residual” is the difference between the simulated and actual outputs of the system, which are then multiplied by the Kalman Gain Kj, which is then fed back into the system. While the schematic represents Kj as a seemingly fixed constant, the Kalman Gain is updated each iteration.

#### Trajectory Planning Algorithm
  Because we are given this task in offline mode, the simulated system has knowledge of its surroundings and the obstacles in this space. The obstacles in this map are represented as discrete points on a 2D plane, which include all other possible points our robot can exist in. Therefore, a function is created to take in the map as an input and iterate through the obstacles in order to generate an ideal output path to avoid these points and minimize travel distance. The output path is then fed to the simulation to drive the car through various waypoints to achieve the goal of getting from the starting point to the endpoint.
  
### Mathematical Formulation
#### Kalman Filter
The Mathematical Formulation for the Kalman Filter is derived from the basic Newtonian equations of motion:


Equation 1: Basic physics of motion. Vf  represents the instantaneous velocity at a certain time, given a prior velocity V0 sampled at t seconds before Vf. Similarly Xf is the difference in displacement from X0 given a time t.

These equations are then reformatted to derive the state-space model of the system:

Equation 2: State space model of the system. By using matrices, these complex equations can be approximated with the basic state space model x_new = Ax + Bu, y_new = Cx + Du. ‘D’ in this case is just zero. Similarly, covariance can be propagated with these equations such that covariance P = A * P * AT + Q (process noise).

Kalman filtering is then performed through the following equations:


Equation 3: The Kalman gain, Kk in equation 1 above, is calculated by working with the Covariance matrix of the inputs, propagated to represent the measurements added with some measurement noise R. H in these equations is equivalent to our C matrix. Through Kalman gain, the next state and covariance can thus be calculated.

#### Car modeling

Equation 4: Those formula are used in the  real life of car moving. According to to the formula between theta and arc length, arc length equals the theta multiply by radius, I found the the velocity of the car. When the car is moving on a plan, its velocity equal to wheel's angular multiply by wheel's radius. Then we can use velocity of car to find the velocity in x and y axis. In the same way, we can use the angular velocity of wheel to find the arc length of the car turned and transfer it to car's angular velocity. Let the car turns with a theta angle, left wheel has a positive velocity, and the right wheel has a negative velocity.


## Results:
Using MATLAB, we were able to successfully implement a program to simulate hardware and software aspects of the vehicular system. By writing functions to model interactions between individual components of the system (such as the microcontroller unit, infrared sensor, inertial measurement unit, and rotational servos), and slaving the calling of each function to a centralized function in lieu of an onboard clock, we were able to mimic the process of dead-reckoning to determine the vehicle’s position relative to its starting position, calculate a new vector towards its goal position, and monitor its progress using several metrics - real, filtered, and unfiltered.      

Figure 2: Graph plotting the filtered state (in green) versus the actual state (in red) and the unfiltered state (in blue), with additive noise represented as a Gaussian with zero mean and variance proportional to the magnitude of the signal scaled by a constant. The current operational state is updated every 50 ms. The state is represented as {x, y} coordinates relative to the base reference frame. 

As evidenced by the above figure (Figure 1), the filtered state using a Kalman filter had some linear offset error which worsened as traveling distance increased, likely due to unoptimized selection of measurement and process noise estimations. A consistent overestimation of velocity in the x and y directions is the probable cause of the discrepancy between the filtered state estimation and the real trajectory. The sampling rate of 20 Hz (every 50 ms) was clearly frequent enough at this scale that the unfiltered state estimation was roughly close to that of the real state. The elapsed time between state updates was brief enough to mitigate sensor noise. 
On the other hand, as evidenced by the figure below (Figure 2), a noticeable discrepancy between the real state and unfiltered state estimation begins to arise as this elapsed time increases to 100 ms, or when the system updates at 10 Hz. It is not at all unreasonable to assume that this discrepancy between the real state and the unfiltered state will continue to worsen as the elapsed time increases, while the filtered state maintains its given offset.      

Figure 3: Graph plotting the filtered state (in green) versus the actual state (in red) and the unfiltered state (in blue), with additive noise represented as a Gaussian with zero mean and variance proportional to the magnitude of the signal scaled by a constant. The current operational state is updated every 100 ms.

Figure 4: Graph plotting the filtered state (in green) versus the actual state (in red) and the unfiltered state (in blue), with additive noise represented as a Gaussian with zero mean and variance proportional to the magnitude of the signal scaled by a constant. The current operational state is updated every 200 ms.

As expected, as the sampling rate decreases, the error in the unfiltered state estimation increases while the filtered state estimation using a Kalman Filter maintains its accuracy. 

Figure 5: Graph plotting the filtered state (in green) versus the actual state (in red) and the unfiltered state (in blue), for a trajectory plotted between predefined waypoints set at {10, 10}, {15, 5}, {20, 10}, {25, 5}, {30, 10}, and {35, 5}. The current operational state is updated every 50 ms. 

In the figure below (Figure 6), we show the block diagram systems view of our simulation.

Figure 6: Block diagram representation of our simulated model.

## Conclusion
In conclusion, based on the results as shown in Figures 2 through 5, we were able to successfully generate a simulation that can be modified to more accurately represent the motion of a two-wheeled vehicle with additional experiments. Due to the modular design of our MATLAB code, it will be easy to integrate additional improvements and modifications as necessary to account for unforeseen nonlinearities or better models of noise to result in a more accurate prediction of our physical model’s motion.    
