[video1]: ./MPC_video.mov

# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

##Here's a [link to my video result](./MPC_video.mov)
As you will see, I was able to achieve speeds over 100 mph. Even driving on an ideal line was possible. The car breaks before strong turns and makes smooth turns.
The only little mistake I still recognize is, that the predicted and fitted waypoints sometimes shift because of the signal delay. Alltogether I am very happy with my solution.

## The Model

I was using a standard dynamic model, adding tan(-delta) and effects of acceleration:
            
    // kinematic model
    const auto px1_f = px0 + v0 * CppAD::cos(psi0) * dt;
    const auto py1_f = py0 + v0 * CppAD::sin(psi0) * dt;
    const auto psi1_f = psi0 + (v0 * CppAD::tan(-delta0) / Lf) * dt + (a0 * CppAD::tan(-delta0) / (2*Lf)) * dt * dt;
    const auto v1_f = v0 + a0 * dt;
    const auto cte1_f = ((py_desired-py0) + (v0 * CppAD::sin(epsi0) * dt));//py_desired - py1_f;
    const auto epsi1_f = ((psi0 - psi_desired) + (v0 * CppAD::tan(-delta0) / Lf) * dt);//-CppAD::atan(psi_desired) + psi1_f;

I used the following COST Functions, to make the drive smooth and fast:
    
    //*********************************************************
    //* COST CALCULATION
    //*********************************************************

    fg[0] = 0.0;

    for (int i = 0; i < N; ++i) {

        const auto cte = x[ID_FIRST_cte + i];
        const auto epsi = x[ID_FIRST_epsi + i];
        const auto v = x[ID_FIRST_v + i] - VELOCITY_MAX;

        fg[0] += (W_cte * pow(cte,2) + W_epsi * pow(epsi,2) + W_v * pow(v,2));
    }

    for (int i = 0; i < N - 1; ++i) {

        const auto delta = x[ID_FIRST_delta + i];
        const auto a = x[ID_FIRST_a + i];

        fg[0] += (W_delta * pow(delta,2) + W_a * pow(a,2));
    }

    for (int i = 0; i < N - 2; ++i) {

        const auto ddelta = x[ID_FIRST_delta + i + 1] - x[ID_FIRST_delta + i];
        const auto da = x[ID_FIRST_a + i + 1] - x[ID_FIRST_a + i];

        fg[0] += (W_ddelta * pow(ddelta,2) + W_da * pow(da,2));
    }
    
I tuned the weights for the cost in following manner:

    // COST WEIGHTS 
    const double W_cte = 15000.0;
    const double W_epsi = 200000.0;
    const double W_v = 8.0;
    const double W_delta = 1000000.0;
    const double W_a = 20.0;
    const double W_ddelta = 100000.0;
    const double W_da = 100000000.0;

## Timestep Length and Elapsed Duration

For caclulation reasons and to be precise enough I chose to take 10 calculation steps with a time difference of 0.3 seconds;

    const int N = 10; // number of predicted states
    const double dt = 0.3; // time per state

Since the car drives roundabout 100 mph which is ca. 44 m/s. The Prediction is about 170 meters long. The reason for such a forward looking prediction is to recognize curves early and brake.

These Parameters where tuned in following way:
I started with 20 observations in 0.1 second distance and gradually increased the number of observations, while tuning the weights. After I got a good feeling, I increased the times per state to 0.3 for computanional reasons.
If I chose even bigger dts the algorithm was not as precise.

## Polynomial Fitting and MPC Preprocessing

First I transform the received waypoints into the vehicle coordinate system and then I do a polynomial fit to get a smooth curve trough all waypoints.
That curve later serves as the path I want my vehicle to follow.

    //**************************************************************
    //* TRANSFORM WAYPOINTS INTO VEHICLE SYSTEM
    //**************************************************************
    const int NUMBER_OF_WAYPOINTS = points_xs.size();
    Eigen::VectorXd waypoints_xs(NUMBER_OF_WAYPOINTS);
    Eigen::VectorXd waypoints_ys(NUMBER_OF_WAYPOINTS);

    for(int i = 0; i < NUMBER_OF_WAYPOINTS; ++i) {

      const double dx = points_xs[i] - delayed_px;
      const double dy = points_ys[i] - delayed_py;

      waypoints_xs[i] = dx * cos(-delayed_psi) - dy * sin(-delayed_psi);
      waypoints_ys[i] = dy * cos(-delayed_psi) + dx * sin(-delayed_psi);
    }

    //**************************************************************
    //* POLYNOMAL FIT WITH 2ND ORDER
    //**************************************************************
    const int ORDER = 2;
    auto K = polyfit(waypoints_xs, waypoints_ys, ORDER);


## Model Predictive Control with Latency

Dealing with the latency was quite easy. I chose to take the received state and calculate where the vehicle would be after the delayed time.
Afterwoods I started transforming the waypoints, fitting the polynomial and solving the MPC.

    //**************************************************************
    //* CONVERT TO DELAYED STATE
    //**************************************************************

    const double delay = 100; //ms
    const double Lf = 2.67;

    //Using the kinematic model
    const double delayed_px = px + speed_mps * cos(psi) * delay/1000;
    const double delayed_py = py + speed_mps * sin(psi) * delay/1000;
    const double delayed_psi = psi + (speed_mps * tan(-delta) / Lf) * delay/1000 + ( (a * tan(-delta) / (2*Lf)) * pow(delay/1000,2));
    const double delayed_v = speed_mps + a * delay/1000;
