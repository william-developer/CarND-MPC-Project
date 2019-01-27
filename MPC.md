# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## MPC Model
Model Predictive Control used optimizer to find the control inputs
and minimized the cost function.The model was consist of the following part:
* states: used for the model were - x coordinate of vehicle, y coordinate of vehicle, orientation, velocity, cross track error and orientation error
* actuators:calculated by controller were steer angle and throttle
* update equations:calculated the state of the vehicle step by step.
The code was:
```
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
```

## Timestep Length and Elapsed Duration (N & dt)

I chosen N=10 and dt=0.1.So the T=N*dt=1 second.
Too large N value maked the processing time larger and response slower, which created problems for my vehicle to take sharp turns
while too less value of N also maked my vehicle to move erraticaly because model had no idea of the future state to give current actuator values.
so N was set 10.
Large values of dt resulted in less frequent actuations, which made it harder to accurately approximate a continuous reference trajectory,
while too less values of dt resulted in high actuate frequency,which made vehicle to run not very smoothly.
so dt was set 0.1.


## Polynomial Fitting and MPC Preprocessing
* Polynomial Fitting:fitting the polynomial firstly I computed waypoints in vehicles frame of refrence.
Then I used polyfit function to fit the third order polynomial to the points.
Waypoints in vehicles frame of reference were required for plotting line in the simulator.
The code was:
```
auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);
```
* MPC Preprocessing


## Model Predictive Control with Latency
Latency was set 0.1 seconds for the system.
To take its effect vehicle position 0.1 seconds in future was predicted using state model equations.
Because MPC model will give actuations to the vehicle after 0.1 seconds,
the initial state should be after 0.1 seconds to get the correct optimum control values.
Since we calculated waypoints from vehicles frame of reference,
future point of vehicle was also calculated from vehicle frame of reference.
The code was:
```
px  += v * cos(psi) * latency;
py  += v * sin(psi) * latency;
psi -= ( v / Lf ) * delta * latency;
v   += a * latency;
```