// Kalman filter 1D constant velocity example.
//
// Constructs a KalmanFilter object and tracks a target traveling with
// 1-dimensional constant velocity. Steps the filter 4 times and prints
// predicted state values.
//
// Compile:
//  g++ -std=c++20 -I../include -I/usr/local/lib/boost_1_75_0
//      ex_1d_const_vel.cpp -o ex_1d_const_vel

#include <KalmanFilter.hpp>
#include <iostream>

using namespace track;

int main()
{
    double dt = 0.1; // sec
    KalmanFilter<double> kf("1d_const_vel", dt);

    // Create measurement values.
    std::vector<double> x_vals = {0.001, 0.998, 2.003, 2.999};
    std::vector<double> vx_vals = {1.021, 0.998, 1.101, 0.999};

    // Initialize state vector.
    std::vector<double> init_state_vals = {0, 1};
    Matrix<double> state0(init_state_vals,2,1);
    kf.state = state0;

    // Initialize filter and validate properties.
    kf.init();

    for (int k = 0; k < x_vals.size(); k++)
    {
        // Get a 2x1 measurement vector.
        std::vector<double> y_vals = {x_vals[k], vx_vals[k]};
        Matrix<double> y(y_vals,2,1);
        // Step the filter.
        kf.update(y);
        kf.predict();
        // Get the predicted state. The state vector is a column vector
        // containing position and velocity, respectively.
        Matrix<double> x = kf.state;
        std::cout << "x = " << x(0,0) << std::endl;
        std::cout << "v_x = " << x(1,0) << "\n" << std::endl;
    }

    return 0;
}
