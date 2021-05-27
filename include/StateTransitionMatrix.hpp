#ifndef STATETRANSITIONMATRIX_H_
#define STATETRANSITIONMATRIX_H_

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/storage.hpp>
#include <vector>
#include <stdexcept>
#include "TrackingMatrix.hpp"
#include "matrix_utils.hpp"

// Provides a state transition matrix.
// Matrix elements may be provided as a std::vector or a
// boost::numeric::ublas::matrix.
//
// The state transition matrix is a real-valued M-by-M matrix where
// M is the number of states.
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
//          boost::numeric::ublas::matrix<double> m(4,4);
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
    if (num_rows != num_cols)
    {
        throw std::invalid_argument("State transition matrix must be a square \
            matrix.");
    }
    if (num_elems != num_rows*num_cols)
    {
        throw std::invalid_argument("Number of matrix elements must equal \
        num_rows * num_cols.");
    }
}

// Constructs from vector and matrix dimensions.
template <class T>
StateTransitionMatrix<T>::StateTransitionMatrix(std::vector<T>& v, int num_rows,
    int num_cols)
{
    validate_dimensions(v.size(), num_rows, num_cols);
    boost::numeric::ublas::matrix<T> m = create_matrix_from_vector(v, num_rows,
        num_cols);
    this->data = m;
}

// Constructs from matrix.
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
