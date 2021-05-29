#define BOOST_TEST_MODULE Matrix
#include <boost/test/unit_test.hpp>

#include <Matrix.hpp>

// -----------------------------------------------------------------------------
// Test constructors.

// Smoke test for default constructor.
BOOST_AUTO_TEST_CASE(testDefaultConstructor)
{
    track::Matrix<> m;
    BOOST_CHECK(true);
}

// Test constructor that initializes to a matrix of all zeros.
BOOST_AUTO_TEST_CASE(testZeroConstructor)
{
    track::Matrix<double> m(2,2);
    BOOST_CHECK_EQUAL(m(0,0), 0);
    BOOST_CHECK_EQUAL(m(0,1), 0);
    BOOST_CHECK_EQUAL(m(1,0), 0);
    BOOST_CHECK_EQUAL(m(1,1), 0);
}

// Create a matrix of type int and verify that its contents have the expected
// type.
BOOST_AUTO_TEST_CASE(testIntType)
{
    track::Matrix<int> m(2,2);
    BOOST_CHECK_EQUAL(m(0,0), (int)0);
    BOOST_CHECK_EQUAL(m(0,1), (int)0);
    BOOST_CHECK_EQUAL(m(1,0), (int)0);
    BOOST_CHECK_EQUAL(m(1,1), (int)0);
}

// Test ublas::matrix constructor.
BOOST_AUTO_TEST_CASE(testMatConstructor)
{
    boost::numeric::ublas::identity_matrix<double> m0(2,2);
    track::Matrix<double> m(m0);
    BOOST_CHECK_EQUAL(m(0,0), 1);
    BOOST_CHECK_EQUAL(m(0,1), 0);
    BOOST_CHECK_EQUAL(m(1,0), 0);
    BOOST_CHECK_EQUAL(m(1,1), 1);
}


// Use vector contructor and assert that matrix contents are correct.
BOOST_AUTO_TEST_CASE(testVectorConstuctor)
{
    std::vector<double> v = {1,2,3,4};
    track::Matrix<double> m(v,2,2);
    BOOST_CHECK_EQUAL(m(0,0), 1);
    BOOST_CHECK_EQUAL(m(0,1), 2);
    BOOST_CHECK_EQUAL(m(1,0), 3);
    BOOST_CHECK_EQUAL(m(1,1), 4);
}


// -----------------------------------------------------------------------------
// Test operators.

// Take the product of the 2-by-2 identity matrix and matrix M2. Check that the
// result equals M2.
BOOST_AUTO_TEST_CASE(testMatrixMultiply)
{
    std::vector<double> v1 = {1,0,0,1};
    track::Matrix<double> m1(v1,2,2);
    std::vector<double> v2 = {1,2,3,4};
    track::Matrix<double> m2(v2,2,2);
    track::Matrix<double> res = m1 * m2;
    BOOST_CHECK(res.num_rows() == 2);
    BOOST_CHECK(res.num_cols() == 2);
    BOOST_CHECK_EQUAL(res(0,0), 1);
    BOOST_CHECK_EQUAL(res(0,1), 2);
    BOOST_CHECK_EQUAL(res(1,0), 3);
    BOOST_CHECK_EQUAL(res(1,1), 4);
}

// Check that operator () indexes the matrix value as expected.
BOOST_AUTO_TEST_CASE(testParens)
{
    std::vector<double> v1 = {1,0,0,1};
    track::Matrix<double> m(v1,2,2);
    BOOST_CHECK_EQUAL(m(0,0), 1);
    BOOST_CHECK_EQUAL(m(0,1), 0);
}

// Test matrix transpose operation.
BOOST_AUTO_TEST_CASE(testTranspose)
{
    std::vector<double> v1 = {1,2,3,4};
    track::Matrix<double> m(v1,2,2);
    track::Matrix<double> res = m.transpose();
    BOOST_CHECK_EQUAL(res(0,0), 1);
    BOOST_CHECK_EQUAL(res(1,0), 2);
    BOOST_CHECK_EQUAL(res(0,1), 3);
    BOOST_CHECK_EQUAL(res(1,1), 4);
}
