#define BOOST_TEST_MODULE KalmanFilter
#include <boost/test/unit_test.hpp>

#include <KalmanFilter.hpp>
#include <random>

// Test default constructor, default type.
BOOST_AUTO_TEST_CASE(testDefaultConstrutor)
{
    track::KalmanFilter<> kf;
    BOOST_CHECK(true);
}

// Check that init() executes succesfully after constructing default KF.
BOOST_AUTO_TEST_CASE(testInit)
{
    track::KalmanFilter<double> kf;
    kf.init();
    BOOST_CHECK(true);
}

// Test string constructor, 1D const vel motion.
BOOST_AUTO_TEST_CASE(testStringConstruct1DConstVel)
{
    double dt = 0.1;
    track::KalmanFilter<double> kf("1d_const_vel", dt);
    kf.init();
    // Validate state transition matrix properties.
    BOOST_CHECK(kf.state_transition_matrix.num_rows() == 2);
    BOOST_CHECK(kf.state_transition_matrix.num_cols() == 2);
    BOOST_CHECK_EQUAL(kf.state_transition_matrix(0,0), 1);
    BOOST_CHECK_EQUAL(kf.state_transition_matrix(0,1), dt);
    BOOST_CHECK_EQUAL(kf.state_transition_matrix(1,0), 0);
    BOOST_CHECK_EQUAL(kf.state_transition_matrix(1,1), 1);
    // Validate measurement matrix properties.
    BOOST_CHECK(kf.measurement_matrix.num_rows() == 2);
    BOOST_CHECK(kf.measurement_matrix.num_cols() == 2);
    BOOST_CHECK_EQUAL(kf.measurement_matrix(0,0), 1);
    BOOST_CHECK_EQUAL(kf.measurement_matrix(0,1), 0);
    BOOST_CHECK_EQUAL(kf.measurement_matrix(1,0), 0);
    BOOST_CHECK_EQUAL(kf.measurement_matrix(1,1), 1);
    // Validate process noise matrix properties.
    BOOST_CHECK(kf.process_noise.num_rows() == 2);
    BOOST_CHECK(kf.process_noise.num_cols() == 2);
    BOOST_CHECK_EQUAL(kf.process_noise(0,0), 1);
    BOOST_CHECK_EQUAL(kf.process_noise(0,1), 0);
    BOOST_CHECK_EQUAL(kf.process_noise(1,0), 0);
    BOOST_CHECK_EQUAL(kf.process_noise(1,1), 1);
    // Validate measurement noise matrix properties.
    BOOST_CHECK(kf.measurement_noise.num_rows() == 2);
    BOOST_CHECK(kf.measurement_noise.num_cols() == 2);
    BOOST_CHECK_EQUAL(kf.measurement_noise(0,0), 1);
    BOOST_CHECK_EQUAL(kf.measurement_noise(0,1), 0);
    BOOST_CHECK_EQUAL(kf.measurement_noise(1,0), 0);
    BOOST_CHECK_EQUAL(kf.measurement_noise(1,1), 1);
}

// Step KF after initializing a 1D const. vel. filter.
BOOST_AUTO_TEST_CASE(test1DConstVelSmoke)
{
    double dt = 0.1;
    track::KalmanFilter<double> kf("1d_const_vel", dt);
    kf.init();
    // Get a 2x1 measurement, all zeros.
    track::Matrix<double> y("zeros",2,1);
    kf.init();
    // Step the filter.
    kf.update(y);
    kf.predict();
    // If we made it this far, the test passes.
    BOOST_CHECK(true);
}

// Simulate 2D const vel tracking. Assert that the estimated trajectory
// approximately equals the true trajectory.
BOOST_AUTO_TEST_CASE(test2DConstVelSim)
{
    // Set up Gaussian random number generator.
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0,1.0);
    // Filter period (sec).
    double dt = 0.1;
    // Create ground truth trajectory.
    int nsamp = 100; // Num. times the filter will be stepped.
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
    // Set up filter.
    track::KalmanFilter<> kf("2d_const_vel", dt);
    // Initialize state vector. The state vector contains x, v_x, y, v_y.
    std::vector<double> init_state_vals = {-1.4, 1.5, 1.2, 0.25};
    track::Matrix<double> state0(init_state_vals,4,1);
    kf.state = state0;
    kf.init();
    // Create vectors that will hold estimated trajectory results.
    std::vector<double> x_est(nsamp);
    std::vector<double> y_est(nsamp);
    for (int k = 0; k < nsamp; k++)
    {
        // Get a measurement vector.
        double x_meas = x_true[k] + distribution(generator);
        double y_meas = y_true[k] + distribution(generator);
        std::vector<double> meas_vec = {x_meas, y_meas};
        track::Matrix<double> y(meas_vec,2,1);
        // Step the filter.
        kf.update(y);
        kf.predict();
        // Get the predicted state. The state vector is a column vector
        // containing position and velocity, respectively.
        track::Matrix<double> x = kf.state;
        x_est[k] = x(0); // First elem. of state vector is x position.
        y_est[k] = x(2); // Second elem. of state vector is y position.
    }
    // Check that the estimated trajectory approx. equals the ground truth.
    // TODO:
    //     *** Add linear regression and check the fitted params. ***
    double x_final_act = x_est[99];
    double x_final_exp = x_true[99];
    double y_final_act = y_est[99];
    double y_final_exp = y_true[99];
    BOOST_CHECK_CLOSE(x_final_act, x_final_exp, 20); // 20% tolerance
    BOOST_CHECK_CLOSE(y_final_act, y_final_exp, 20);
}
