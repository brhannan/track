# track
**track** is a C++ state estimation and tracking library.

## Example

Set up a Kalman filter for tracking a 2D constant-velocity target.
The filter runs at 10 Hz.

See /examples/ex_2d_const_vel.cpp.

```cpp
#include <KalmanFilter.hpp>
#include <random>
#include <iostream>

using namespace track;

int main()
{
    // Set up Gaussian random number generator.
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0,1.0);

    // Filter period (sec).
    double dt = 0.1;

    // Create ground truth trajectory.
    int nsamp = 100;
    std::vector<double> x_true(nsamp);
    std::vector<double> vx_true(nsamp);
    std::vector<double> y_true(nsamp);
    std::vector<double> vy_true(nsamp);
    // Specify true initial conditions.
    double x0 = 0;
    double y0 = 0;
    double vx0 = 1;
    double vy0 = 1;
    for (int n = 0; n < nsamp; n++)
    {
        // Sample the Gaussian RNG to add noise to truth dyamics.
        x_true[n] = x0 + vx0*n*dt + distribution(generator);
        vx_true[n] = vx0 + distribution(generator);
        y_true[n] = y0 + vy0*n*dt + distribution(generator);
        vy_true[n] = vy0 + distribution(generator);
    }

    // Set up Kalman filter. By default, the 2d_const_vel filter is configured
    // to measure x position and y position only (velocities are not measured).
    // This can be changed by specifying custom values for
    // kf.measurement_matrix.
    KalmanFilter<double> kf("2d_const_vel", dt);

    // Initialize state vector. The state vector contains x, v_x, y, v_y.
    std::vector<double> init_state_vals = {-4.4, 1.5, 5.2, -1.25};
    Matrix<double> state0(init_state_vals,4,1);
    kf.state = state0;

    // Initialize the filter and validate its properties.
    kf.init();

    for (int k = 0; k < nsamp; k++)
    {
        // Measure the target's position. Add noise to the true position.
        double x_meas = x_true[k] + distribution(generator);
        double y_meas = y_true[k] + distribution(generator);
        std::vector<double> meas_vec = {x_meas, y_meas};
        Matrix<double> y(meas_vec,2,1);
        // Step the filter.
        kf.update(y);
        kf.predict();
        // Print the predicted state.
        Matrix<double> x = kf.state;
        std::cout << "x = " << x(0) << " v_x = " << x(1) << " y = " << x(2) <<
            " v_y = " << x(3) << std::endl;
    }
}
```

The 2D constant velocity example results are shown below. Truth values are
marked with black dots. Estimated values are shown by the blue circles.

![](https://github.com/brhannan/track/blob/main/docs/images/kf_2d_const_vel.gif)


## Dependencies
[Boost](https://www.boost.org)  

## Test

To run the test suite:
- Make sure that `MY_BOOST_DIR` in test/Makefile contains a valid path.
- `cd` to /test.
- Enter commands `make all` and then `sh runTests.sh`.
