# track
**track** is a C++ state estimation and tracking filter library.

## Examples

Set up a Kalman filter for tracking a target that is expected to
travel in one dimension with constant velocity. The filter runs at 10 Hz.

```cpp
#include <KalmanFilter.hpp>
#include <iostream>

using namespace track;

int main()
{
    double dt = 0.1; // sec
    KalmanFilter<double> kf("1d_const_vel", dt);

    // Initialize state vector to x = 0, v_x = 1.
    std::vector<double> init_state_vals = {0.001, 1.002};
    Matrix<double> state0(init_state_vals,2,1);

    kf.state = state0;
    kf.init(); // Initialize and validate properties.

    // Create a 2x1 measurement vector.
    Matrix<double> y("zeros",2,1);

    // Step the filter once.
    kf.update(y);
    kf.predict();

    // Get the predicted state. The state vector is a column vector containing
    // position and velocity, respectively.
    Matrix<double> x = kf.state;
    std::cout << "x = " << x(0,0) << std::endl;
    std::cout << "v_x = " << x(1,0) << std::endl;

    return 0;
}


```

## Dependencies
[Boost](https://www.boost.org)  

## Test

To run the test suite:
- Make sure that `MY_BOOST_DIR` in test/Makefile contains a valid path.
- `cd` to /test.
- Enter commands `make all` and then `sh runTests.sh`.
