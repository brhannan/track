#define BOOST_TEST_MODULE KalmanFilter
#include <boost/test/unit_test.hpp>

#include <KalmanFilter.hpp>

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
