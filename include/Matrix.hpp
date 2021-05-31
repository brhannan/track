// track::Matrix class.

#ifndef MATRIX_H_
#define MATRIX_H_

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/storage.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <vector>
#include <stdexcept>
#include <string>
#include <matrix_utils.hpp>

#include <iostream>

namespace track
{

// track::Matrix class.
//
// track::Matrix objects can be used for common matrix operations (addition,
// subtraction, multiplication, matrix inverse, etc.).
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
//
//      // Example 3: Initialize a 4-by-4 matrix of all zeros.
//          track::Matrix<double> mat(4,4)
//
//      // Example 4: Create a 3-by-3 identity matrix.
//          track::Matrix<double> mat("identity",3,3);
//          // Note: In addition to "identity", this Matrix constructor also
//          // supports parameters "zeros" and "ones".
//
//      // Example 5: Multiply two matrices.
//          track::Matrix<int> a(3,2);
//          track::Matrix<int> b(2,4);
//          track::Matrix<int> c = a * b;
//
//      // Example 6: Scalar operations.
//          // Scalar addition, subtraction.
//          track::Matrix<double> m(2,2);
//          double x = 5;
//          track::Matrix<double> m1 = m - x;
//          track::Matrix<double> m2 = m + x;
//          // Multiplication, division by a scalar.
//          track::Matrix<double> m3  = x * m;
//          track::Matrix<double> m4 = m / x;
//
//      // Example 7: Matrix inverse.
//          std::vector<double> v {1, 2, 3, 4};
//          track::Matrix<double> m(v,2,2);
//          track::Matrix<double> m_inv = m.inverse();
template <class T = double>
class Matrix
{
public:
    Matrix(std::vector<T>& v, int num_rows, int num_cols);
    Matrix(int num_rows, int num_cols);
    Matrix(boost::numeric::ublas::matrix<T> m);
    Matrix(std::string mat_type, int num_rows, int num_cols);
    Matrix();

    boost::numeric::ublas::matrix<T> data;

    // Index into a matrix.
    T operator()(const int row_ix, const int col_ix)
    {
        return data(row_ix, col_ix);
    }

    // Index into a vector.
    T operator()(const int ix)
    {
        int ix1, ix2;
        int nrow = num_rows();
        int ncol = num_cols();
        if ( nrow>1 & ncol>1 )
        {
            throw std::invalid_argument("Two indices must be provided when "
                "matrix does not contain a vector.");
        }
        if (nrow>1)
        {
            ix1 = ix;
            ix2 = 0;
        }
        else
        {
            ix1 = 0;
            ix2 = ix;
        }
        if ( ix1 > data.size1() )
        {
            throw std::invalid_argument("Index is out of range.");
        }
        if ( ix2 > data.size2() )
        {
            throw std::invalid_argument("Index is out of range.");
        }
        return data(ix1,ix2);
    }

    // Matrix product.
    Matrix<T> operator*(const Matrix<T>& M)
    {
        using namespace boost::numeric;
        return Matrix<T>( ublas::prod(data, M.data) );
    }

    // Matrix addition.
    Matrix<T> operator+(const Matrix<T>& M)
    {
        return Matrix<T>(data + M.data);
    }

    // Matrix subtraction.
    Matrix<T> operator-(const Matrix<T>& M)
    {
        return Matrix<T>(data - M.data);
    }

    // Unary -.
    Matrix<T> operator-()
    {
        return Matrix<T>(-data);
    }

    // Transposes a matrix.
    Matrix<T> transpose()
    {
        using namespace boost::numeric;
        ublas::matrix<T> data_transposed = ublas::trans(data);
        Matrix<T> m(data_transposed);
        return  m;
    }

    // Calculates matrix inverse using LU inversion.
    Matrix<T> inverse()
    {
        using namespace boost::numeric;
        if (num_rows() != num_cols())
        {
            throw std::runtime_error("Cannot take inverse of non-square "
                "matrix.");
        }
        // TODO: Error if non-singular.
        // Initialize output.
        Matrix<T> res(data.size1(),data.size2());
        bool inv_status = inverse_impl(data,res.data);
        if (!inv_status)
        {
            throw std::runtime_error("Unable to calculate matrix inverse.");
        }
        return res;
    }

    // Gets number of matrix rows.
    int num_rows()
    {
        return data.size1();
    }

    // Gets number of matrix columns.
    int num_cols()
    {
        return data.size2();
    }

    void validate_dimensions(int num_elems, int num_rows, int num_cols);

private:
    static bool inverse_impl(const boost::numeric::ublas::matrix<T>& input,
        boost::numeric::ublas::matrix<T>& inverse);
};

// Validates matrix dimensions.
// num_elems is the actual number of elements; that is, the number of
// elements in the container that holds the values to be copied to
// member variable data. num_rows and num_cols specify the number of
// rows/cols of the desired matrix.
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
    if (num_elems != num_rows*num_cols)
    {
        throw std::invalid_argument("Number of matrix elements must equal "
            "num_rows * num_cols.");
    }
}

// Constructs from vector and matrix dimensions.
template <class T>
Matrix<T>::Matrix(std::vector<T>& v, int num_rows,
    int num_cols)
{
    validate_dimensions(v.size(), num_rows, num_cols);
    boost::numeric::ublas::matrix<T> m = track::create_matrix_from_vector(v,
        num_rows, num_cols);
    data = m;
}

// Constructs a num_rows-by-num_cols matrix of zeros.
template <class T>
Matrix<T>::Matrix(int num_rows, int num_cols)
{
    boost::numeric::ublas::zero_matrix<T> m(num_rows, num_cols);
    data = m;
}

// Constructs from matrix.
template <class T>
Matrix<T>::Matrix(boost::numeric::ublas::matrix<T> m)
{
    int num_rows = m.size1();
    int num_cols = m.size2();
    int mat_size = num_rows * num_cols;
    validate_dimensions(mat_size, num_rows, num_cols);
    data = m;
}

// Initializes matrix using string "identity", "ones", or "zeros".
template <class T>
Matrix<T>::Matrix(std::string mat_type, int num_rows, int num_cols)
{
    boost::numeric::ublas::matrix<T> m(num_rows,num_cols);
    if (mat_type == "identity")
    {
        if ( num_rows != num_cols )
        {
            throw std::invalid_argument("Cannot create non-square identity \
                matrix.");
        }
        boost::numeric::ublas::identity_matrix<T> m(num_rows);
        data = m;
    }
    else if (mat_type == "ones")
    {
        std::vector<T> v(num_rows*num_cols, 1);
        boost::numeric::ublas::matrix<T> m =
            track::create_matrix_from_vector(v, num_rows, num_cols);
        data = m;
    }
    else if (mat_type == "zeros")
    {
        boost::numeric::ublas::zero_matrix<T> m(num_rows, num_cols);
        data = m;
    }
    else
    {
        throw std::invalid_argument("Invalid mat_type value. mat_type is "
            "expected to equal \"identity\", \"ones\" or \"zeros\".");
    }
}

// Default constructor.
template <class T>
Matrix<T>::Matrix() {}

// Implements scalar*matrix product.
template <class T>
Matrix<T> operator*(const T left, const Matrix<T>& right)
{
    return Matrix<T>( left * right.data );
}

// Implements matrix*scalar product.
template <class T>
Matrix<T> operator*(const Matrix<T>& left, const T right)
{
    return Matrix<T>( right * left.data );
}

// Implements division matrix/scalar.
template <class T>
Matrix<T> operator/(const Matrix<T>& left, const T right)
{
    return Matrix<T>( left.data / right );
}

// Implements scalar+matrix product.
template <class T>
Matrix<T> operator+(const T left, const Matrix<T>& right)
{
    Matrix<T> m(right);
    Matrix<T> M0(right);
    for (auto i=0; i<M0.num_rows(); i++)
    {
        for (auto j=0; j<M0.num_cols(); j++)
        {
            m.data(i,j) = M0(i,j) + left;
        }
    }
    return m;
}

// Implements matrix+scalar product.
template <class T>
Matrix<T> operator+(const Matrix<T>& left, const T right)
{
    Matrix<T> m(left);
    Matrix<T> M0(left);
    for (auto i=0; i<M0.num_rows(); i++)
    {
        for (auto j=0; j<M0.num_cols(); j++)
        {
            m.data(i,j) = M0(i,j) + right;
        }
    }
    return m;
}

// Calculates inverse of input. Result is written to parameter inverse. Output
// is false if LU factorization fails.
template <class T>
bool Matrix<T>::inverse_impl(const boost::numeric::ublas::matrix<T>& input,
    boost::numeric::ublas::matrix<T>& inverse)
{
    using namespace boost::numeric::ublas;
    typedef permutation_matrix<std::size_t> pmatrix;
    matrix<T> A(input);
    pmatrix pm(A.size1());
    // LU decomposition
    int res = lu_factorize(A, pm);
    if (res != 0)
    {
        return false;
    }
    inverse.assign(identity_matrix<T>(A.size1()));
    lu_substitute(A, pm, inverse);
    return true;
}

} // namespace track

#endif // MATRIX_H_
