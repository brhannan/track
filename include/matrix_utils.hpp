#ifndef MATRIX_UTILS_H_
#define MATRIX_UTILS_H_

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/storage.hpp>
#include <vector>
#include <stdexcept>
#include <Matrix.hpp>

namespace track
{

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

} // namespace track

#endif // MATRIX_UTILS_H_
