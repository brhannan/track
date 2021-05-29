#include <Matrix.hpp>


namespace track
{

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

// // Constructs from vector and matrix dimensions.
// template <class T>
// Matrix<T>::Matrix(std::vector<T>& v, int num_rows,
//     int num_cols)
// {
//     validate_dimensions(v.size(), num_rows, num_cols);
//     boost::numeric::ublas::matrix<T> m = create_matrix_from_vector(v, num_rows,
//         num_cols);
//     data = m;
// }
//
// // Constructs from matrix.
// template <class T>
// Matrix<T>::Matrix(boost::numeric::ublas::matrix<T> m)
// {
//     int num_rows = m.size1;
//     int num_cols = m.size2;
//     int mat_size = num_rows * num_cols;
//     validate_dimensions(mat_size, num_rows, num_cols);
//     data = m;
// }
//
// // Default constructor.
// template <class T>
// Matrix<T>::Matrix() {}

} // namespace track
