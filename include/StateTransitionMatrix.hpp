#ifndef STATETRANSITIONMATRIX_H_
#define STATETRANSITIONMATRIX_H_

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/storage.hpp>
#include <vector>
#include <stdexcept>
#include "TrackingMatrix.hpp"

// Provides a state transition matrix.
// Matrix elements may be provided as a std::vector or a
// boost::numeric::ublas::matrix.
//
// The state transition matrix is a real-valued N-by-M matrix where N is the
// number of elements of the measurement vector and M is the number of states.
//
// Examples:
//
//      // Example 1: Create a StateTransitionMatrix from a vector of matrix
//      // elements.
//          std::vector<double> v {1, 2, 3, 4};
//          // Reshape v into a 2-by-2 matrix. v is wrapped row-wise.
//          StateTransitionMatrix<double> stm(v,2,2);
//
//      // Example 2: Initialize with a ublas::matrix.
//          boost::numeric::ublas::matrix<double> m(3,4);
//          StateTransitionMatrix<double> stm(m);
template <class T>
class StateTransitionMatrix : public TrackingMatrix<T>
{
public:
    void validate_dimensions(int num_elems, int num_rows, int num_cols);
    StateTransitionMatrix(std::vector<T>& v, int num_rows, int num_cols);
    StateTransitionMatrix(boost::numeric::ublas::matrix<T> m);
    StateTransitionMatrix();
};

// Returns a m-by-n matrix containing the data specified by vector v. v contains
// the row-unwrapped elements of the matrix.
template <class T, typename F = boost::numeric::ublas::row_major>
boost::numeric::ublas::matrix<T,F> create_matrix_from_vector(const std::vector<T>& v,
    std::size_t m, std::size_t n)
{
    if (m*n != v.size())
    {
        throw std::invalid_argument("Invalid matrix dimensions.");
    }
    boost::numeric::ublas::unbounded_array<T> elems(m*n);
    std::copy(v.begin(), v.end(), elems.begin());
    return boost::numeric::ublas::matrix<T>(m, n, elems);
}

template <class T>
void StateTransitionMatrix<T>::validate_dimensions(int num_elems, int num_rows,
    int num_cols)
{
    if (num_elems <= 0)
    {
        throw std::invalid_argument("Number of elements must be > 0.");
    }
    if (num_rows <= 0)
    {
        throw std::invalid_argument("Number of rows must be > 0.");
    }
    if (num_cols <= 0)
    {
        throw std::invalid_argument("Number of columns must be > 0.");
    }
}

// Constructs STM from vector.
template <class T>
StateTransitionMatrix<T>::StateTransitionMatrix(std::vector<T>& v, int num_rows,
    int num_cols)
{
    validate_dimensions(v.size(), num_rows, num_cols);
    boost::numeric::ublas::matrix<T> m = create_matrix_from_vector(v, num_rows,
        num_cols);
    this->data = m;
}

// Constructs STM from matrix.
template <class T>
StateTransitionMatrix<T>::StateTransitionMatrix(boost::numeric::ublas::matrix<T> m)
{
    int num_rows = m.size1;
    int num_cols = m.size2;
    int mat_size = num_rows * num_cols;
    validate_dimensions(mat_size, num_rows, num_cols);
    this->data = m;
}

// Default constructor.
template <class T>
StateTransitionMatrix<T>::StateTransitionMatrix() {}

#endif // STATETRANSITIONMATRIX_H_
