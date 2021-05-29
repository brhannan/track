#define BOOST_TEST_MODULE KalmanFilter
#include <boost/test/unit_test.hpp>

#include <KalmanFilter.hpp>

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
