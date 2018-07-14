# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Overview
This project implements a Model Predictive Controller(MPC) to control a car in Udacity's simulator.
The simulator sends car telemetry information to the MPC using WebSocket and it receives the steering
angle and throttle. The MPC uses the uWebSockets WebSocket implementation to handle this communication.

## Prerequisites

1. cmake >= 3.5
2. make >= 4.1
3. gcc/g++ >= 5.4
4. Udacity's simulator.
5. Ipopt
6. CppAD

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


## Implementation

## The Model
The car's state is modelled as a Kinematic model defined as below:

1. x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
2. y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
3. psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
4. v[t+1] = v[t] + a[t] * dt
5. cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
6. epsi[t+1] = psi[t+1] - psides[t] + v[t] * delta[t] / Lf * dt

Where:

1. x, y - Car's coordinates,
2. psi - Car's heading direction,
3. v - Car's velocity,
4. cte - Cross-track error,
5. epsi - Orientation error.

In the equations above, Lf is the distance between the car of mass and the front wheels.

The models' output variables are:

1. a - Car's throttle
2. delta - Steering angle.

The objective is to find the acceleration (a) and the steering angle(delta) in the way it will minimize
an objective function that is the combination of different factors:

1. Norm Square of cte,
2. Norm Square of epsi, 
3. Norm Square of difference of speed with reference/desired speed, 
4. Norm Square of the a to force min vaue optimium solution,
5. Norm Square of the delta to force min vaue optimium solution,
6. Norm Square of difference between two consecutive a to penalize sharp throttle changes,
7. Norm Square of difference between two consecutive delta to penalize sharp steering angle changes.

The weights of each of these factors had were tuned manually to obtain a successful track ride without leaving the road.
Particularly, the weights of (g) was chosen to be relatively higher value than others to make a stable drive around 
the edges, and discourage random changes.

## Timestep Length and Elapsed Duration (N & dt)

The number of points (N) and the time interval (dt) define the prediction horizon. 
The number of points impacts the controller performance. I chose the N and dt to keep the horizon
aligned with the waypoints particularly on the turns, thus keeping the horizon small but big enough to
smoothen the optimizer. I found N = 20 and dt = 0.05 seconds to work  for me.

## Polynomial Fitting and MPC Preprocessing
The provided waypoints are transformed to the car coordinate system

1. std::vector<double> points_xs = j[1]["ptsx"];
2. std::vector<double> points_ys = j[1]["ptsy"];
3. const int numWaypoints = points_xs.size();
4. Eigen::VectorXd waypoints_xs(numWaypoints);
5. Eigen::VectorXd waypoints_ys(numWaypoints);
6. Looping over numWaypoints as {
(a)	double dx = points_xs[i] - px;
(b)	double dy = points_ys[i] - py;
(c)	waypoints_xs[i] = dx * cos(-psi) - dy * sin(-psi);
(d)	waypoints_ys[i] = dy * cos(-psi) + dx * sin(-psi);
}


Following, a 3rd-degree polynomial is fitted to the transformed waypoints -

auto coeffs = polyfit(waypoints_xs, waypoints_ys, 3);

The cte (cross track error) and epsi (orientation error) are thus calculated  

1. double cte  = coeffs[0];
2. double epsi = -atan(coeffs[1]);

## Model Predictive Control with Latency
To handle actuator latency, the state values are calculated using the model and the delay interval.
This is implemented by a 2 step approach -

1. First, at current time t=0 the car's states are px=0, py=0, psi=0 right after converting to car coordinate. 
There cte and epsi are calcualted. 

The velocity is converted from mph to mps as
v*=0.44704;

2. Second, the state for dt = latency = 0.1sec are predicted as

(a) psi = delta; 
(b) px = px + v * cos(psi) * latency; 
(c) py = py + v * sin(psi) * latency;
(d) cte= cte + v * sin(epsi) * latency;
(e) epsi = epsi + v * delta * latency/Lf;
(f) psi = psi + v * delta * latency/Lf;
(g) v = v + a * latency;


