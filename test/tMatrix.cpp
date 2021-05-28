#define BOOST_TEST_MODULE Matrix
#include <boost/test/unit_test.hpp>

#include <Matrix.hpp>

BOOST_AUTO_TEST_CASE( smoke )
{
    track::Matrix<double> my_matrix;
    BOOST_CHECK(true);
}
