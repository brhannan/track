// Abstract base class for classes that implement tracking filter matrices.

#ifndef TRACKINGMATRIX_H_
#define TRACKINGMATRIX_H_

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/storage.hpp>
#include <vector>

template <class T>
class TrackingMatrix
{
public:
    boost::numeric::ublas::matrix<T> data;
    // Validates matrix dimensions.
    // num_elems is the actual number of elements; that is, the number of
    // elements in the container that holds the values to be copied to
    // member variable data. num_rows and num_cols specify the number of
    // rows/cols of the desired matrix.
    virtual void validate_dimensions(int num_elems, int num_rows,
        int num_cols) = 0;
    
    T operator()(int row_ix, int col_ix)
    {
        return this->data(row_ix, col_ix);
    }
};

#endif // TRACKINGMATRIX_H_
