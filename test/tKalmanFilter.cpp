#define BOOST_TEST_MODULE KalmanFilter
#include <boost/test/unit_test.hpp>

#include <KalmanFilter.hpp>

BOOST_AUTO_TEST_CASE( smoke )
{
    track::KalmanFilter<double> kf;
    BOOST_CHECK(true);
}
