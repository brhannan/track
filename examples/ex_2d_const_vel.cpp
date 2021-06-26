// Kalman filter 2D constant velocity example.
//
// Constructs a KalmanFilter object and tracks a target traveling with
// two-dimensional constant velocity.
//
// Compile:
//  g++ -std=c++20 -I../include -I/usr/local/lib/boost_1_75_0
//      ex_2d_const_vel.cpp -o ex_2d_const_vel

#include <KalmanFilter.hpp>
#include <random>
#include <iostream>

using namespace track;

int main()
{
    // Set up Gaussian random number generator.
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0,0.5);

    // Filter period (sec).
    double dt = 0.1;

    // Create ground truth trajectory.
    int nsamp = 100;
    std::vector<double> x_true(nsamp);
    std::vector<double> vx_true(nsamp);
    std::vector<double> y_true(nsamp);
    std::vector<double> vy_true(nsamp);
    // Specify initial conditions.
    double x0 = 0;
    double y0 = 0;
    double vx0 = 1;
    double vy0 = 1;
    for (int n = 0; n < nsamp; n++)
    {
        // Sample the Gaussian RNG to add noise to truth dyamics.
        double dx = distribution(generator);
        double dy = distribution(generator);
        double dvx = distribution(generator);
        double dvy = distribution(generator);
        x_true[n] = x0 + n*vx0*dt + dx;
        vx_true[n] = vx0 + dvx;
        y_true[n] = y0 + n*vy0*dt + dy;
        vy_true[n] = vy0 + dvy;
    }

    // Set up Kalman filter. By default, the 2d_const_vel filter is configured
    // to measure x position and y position only (velocities are not measured).
    KalmanFilter<double> kf("2d_const_vel", dt);

    // Initialize state vector. The state vector contains x, v_x, y, v_y.
    std::vector<double> init_state_vals = {-1.4, 1.5, 1.2, 0.25};
    Matrix<double> state0(init_state_vals,4,1);
    kf.state = state0;

    // Initialize the filter and validate its properties.
    kf.init();

    // Set up a place to store the estimated position.

    for (int k = 0; k < nsamp; k++)
    {
        // Get a measurement vector.
        double x_meas = x_true[k] + distribution(generator);
        double y_meas = y_true[k] + distribution(generator);
        std::vector<double> meas_vec = {x_meas, y_meas};
        Matrix<double> y(meas_vec,2,1);
        // Step the filter.
        kf.update(y);
        kf.predict();
        // Get the predicted state. The state vector is a column vector
        // containing position and velocity, respectively.
        Matrix<double> x = kf.state;
        std::cout << "x = " << x(0) << " v_x = " << x(1) << " y = " << x(2) <<
            " v_y = " << x(3) << std::endl;
    }

    return 0;
}
