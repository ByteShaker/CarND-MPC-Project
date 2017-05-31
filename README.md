[video1]: ./MPC_video

# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
###
Here's a [link to my video result](./PID_Sim.mov)
### The Model

The kinematic model we use for this project is a simplification of dynamic model that ignores tire forces, gravity, and mass.The model describes the car state as [*x, y, &#936;, v, cte, e&#936;*]. The *x, y* are 2D cartesian coordinates of the car. *&#936;* is the orientation of the car heading to. *v* is car velocity, *cte* is cross-track error while *e&#936;* is the orientation error.

The car actuator inputs are [*&#948;, a*], where *&#948;* for steering angle and *a* for acceleration. With these two inputs, the kinematic model can be described with following equations:

![kinematicModel](images/model_equations.png)

Here *Lf* is the distance between the center of mass of the vehicle and the front wheels.

### Timestep Length and Elapsed Duration

The MPC prediction horizon *T* is the duration over which future predictions are made. It is determined by the number of timesteps in the horizon *N* and elapsed duration between timesteps *dt*. The *N* determines the number of variables optimized by MPC. It is also the major driver of computational cost. A longer *dt* could result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory. But a shorter *dt* requires larger *N* for a fixed prediction horizon, thus increases computational cost.

Since our goal is to drive the car in about 50 miles per hour on the track, which is about 23 meters per second. We limit the prediction horizon to be about 12 meters, which means our prediction horizon is about 0.5 seconds. Beyond that, the track can change enough that it won't make sense to predict any futher into the future. We start test our implementation with reference velocity from 20 miles per hour, and limit our searh of *N* from 10 - 16, *dt* from 0.025 - 0.075. We gradually increased reference velocity once we obtain a good result and adjust the values again for the testing velocity. Our final choices of *N* is 10, and *dt* is 0.5. These two values seem work well for reference velocity from 20 to 60 miles per hour. Athough this settings works when the car speed go to 80 miles per hour in simulator, it does show high swing on the second turn after the bridge. This show we need to use a small *T* and different *N* and *dt* for speed higher than 80 miles per hour. 

### Polynomial Fitting and MPC Preprocessing

A 3rd order polynomial is fitted to waypoints in car coordinates. The waypoints passed from simulator are in global coordinates. To transform to car coordinates, they are multiplied with a transform matrix. Following code shows the computation of transform matrix:

         Eigen::Matrix3d T;
          T << std::cos(psi), -std::sin(psi), px,
               std::sin(psi),  std::cos(psi), py,
               0,              0,             1;     

### Model Predictive Control with Latency

There is always a delay from MPC computation to the executation of actuation command. For this project, the latency is assumed to be 100 millisecond. To handle the latency, we estimate the car's prospective state based on its current speed and heading direction. The result is used as car's initial state for MPC trajectory computation. Following code show how the propective state is estimated:

          // taking account command's latency
          const double latency = 0.1;
          const double Lf = 2.67;
          const double cur_d = j[1]["steering_angle"];
          const double cur_a = j[1]["throttle"];

          double dx = v*std::cos(cur_d)*latency;  // car should have moved in x-direction for 100 millisec
          double dy = -v*std::sin(cur_d)*latency;
          double dpsi = -(v*cur_d*latency)/Lf;
          double dv = v + cur_a*latency;
          
          cte = polyeval(coeffs, dx);
          epsi = -CppAD::atan(coeffs(1)+coeffs(2)*dx+coeffs(3)*dx*dx);

          state << dx, dy, dpsi, dv, cte, epsi;
          vector<double> rc = mpc.Solve(state, coeffs);

Another approach to handle the latency is to fix the actuation values to previous values for the duration of latency. This approach requires the latency to be close multiple of *dt*. I believe the approach we take is more flexible. But the calulation of the initial position in current implementation is only approximate. For larger latency, we should use a more accurate formula.

## Result and Reflection

Here is a [link](https://www.youtube.com/watch?v=9Wah9d0_8SA) to my final video output:

<p align="center">
    <a href="https://www.youtube.com/watch?v=9Wah9d0_8SA">
        <img src="https://img.youtube.com/vi/9Wah9d0_8SA/0.jpg" alt="video output">
    </a>
</p>

We get much better result using MPC model compare to PID control. The only issue is when we go to very high speed (>80 miles/hour in this case), we'll see a big swing when there are two close turns. This may be due to the cost functions that require constant velocity. For very high speed, we may need a more flexible constraints on velocity and acceleration.