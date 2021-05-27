// Abstract base class for classes that implement tracking filter matrices.

#ifndef MATRIX_H_
#define MATRIX_H_

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/storage.hpp>
#include <vector>
#include <stdexcept>
#include <matrix_utils.hpp>


namespace track
{

// Class track::Matrix allows simple operations with ublas::matrix objects.
// Matrix elements may be provided as a std::vector or a ublas::matrix.
//
// The state transition matrix is a real-valued M-by-M matrix where
// M is the number of states.
//
// Examples:
//
//      // Example 1: Create a StateTransitionMatrix from a vector of matrix
//      // elements.
//          // Specify values for 2x2 identity matrix.
//          std::vector<double> v {1, 0, 0, 1};
//          // Reshape v into a 2-by-2 matrix. v is wrapped row-wise.
//          track::Matrix<double> mat(v,2,2);
//
//      // Example 2: Initialize with a ublas::matrix.
//          // Create an identity_matrix. You can also use ublas::matrix and
//          // manually set its values.
//          boost::numeric::ublas::identity_matrix<double> m(4,4);
//          track::Matrix<double> mat(m);
template <class T = double>
class Matrix
{
public:
    boost::numeric::ublas::matrix<T> data;

    // Validates matrix dimensions.
    // num_elems is the actual number of elements; that is, the number of
    // elements in the container that holds the values to be copied to
    // member variable data. num_rows and num_cols specify the number of
    // rows/cols of the desired matrix.
    void validate_dimensions(int num_elems, int num_rows,
        int num_cols);

    Matrix(std::vector<T>& v, int num_rows, int num_cols);
    Matrix(boost::numeric::ublas::matrix<T> m);
    Matrix();

    T operator()(int row_ix, int col_ix)
    {
        return data(row_ix, col_ix);
    }
};

template <class T>
void Matrix<T>::validate_dimensions(int num_elems, int num_rows, int num_cols)
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
Matrix<T>::Matrix(std::vector<T>& v, int num_rows,
    int num_cols)
{
    validate_dimensions(v.size(), num_rows, num_cols);
    boost::numeric::ublas::matrix<T> m = create_matrix_from_vector(v, num_rows,
        num_cols);
    data = m;
}

// Constructs from matrix.
template <class T>
Matrix<T>::Matrix(boost::numeric::ublas::matrix<T> m)
{
    int num_rows = m.size1;
    int num_cols = m.size2;
    int mat_size = num_rows * num_cols;
    validate_dimensions(mat_size, num_rows, num_cols);
    data = m;
}

// Default constructor.
template <class T>
Matrix<T>::Matrix() {}

} // namespace track

#endif // MATRIX_H_
