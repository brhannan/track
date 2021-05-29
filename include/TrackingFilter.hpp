#ifndef TRACKING_FILTER_H_
#define TRACKING_FILTER_H_

#include <Matrix.hpp>

namespace track
{

// TrackingFilter base class.
template <class T = double>
class TrackingFilter
{
public:
    // Matrix initialization helper functions.
    bool is_mat_empty(track::Matrix<T>& mat);
    void init_from_identity_mat(track::Matrix<T>& mat, int L, T v = 1);
    void init_column_vector(track::Matrix<T>& vec, int L, T v = 0);
};

// Returns true if mat is empty.
template <class T>
bool TrackingFilter<T>::is_mat_empty(track::Matrix<T>& mat)
{
    return (mat.data.size1()==0) && (mat.data.size2()==0);
}

// Initialize a square matrix by scaling an identity matrix of dimension L. The
// matrix is scaled by scalar v. The result is returned to mat.
template <class T>
void TrackingFilter<T>::init_from_identity_mat(track::Matrix<T>& mat, int L,
    T v)
{
    boost::numeric::ublas::identity_matrix<T> ident(L);
    mat.data = v * ident;
}

// Initialize a column vector by scaling a vector of dimension L. All values
// are set to value v (default 0).
template <class T>
void TrackingFilter<T>::init_column_vector(track::Matrix<T>& vec, int L, T v)
{
    boost::numeric::ublas::matrix<T> m(L,1);
    for (auto i=0; i<m.size1(); i++)
    {
        for (auto j=0; j<m.size2(); j++)
        {
            m(i,j) = v;
        }
    }
    vec.data = m;
}

} // namespace track

#endif // TRACKING_FILTER_H_
