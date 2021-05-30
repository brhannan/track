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

// // Create a default KF and check that update() and correct() execute
// // successfully.
// BOOST_AUTO_TEST_CASE(testOneStep)
// {
//     track::KalmanFilter<double> kf;
//     kf.init();
//     track::Matrix<double> y(2,1);
//     kf.predict();
//     kf.update(y);
//     BOOST_CHECK(true);
// }
