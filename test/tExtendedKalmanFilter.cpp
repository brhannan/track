#define BOOST_TEST_MODULE ExtendedKalmanFilter
#include <boost/test/unit_test.hpp>
#include <ExtendedKalmanFilter.hpp>
#include <random>

// Test default constructor, default type.
BOOST_AUTO_TEST_CASE(testDefaultConstrutor)
{
    track::ExtendedKalmanFilter<> kf;
    BOOST_CHECK(true);
}

// // Check that init() executes succesfully after constructing default KF.
// BOOST_AUTO_TEST_CASE(testInit)
// {
//     track::KalmanFilter<double> kf;
//     kf.init();
//     BOOST_CHECK(true);
// }
