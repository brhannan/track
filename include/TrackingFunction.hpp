#ifndef TRACKINGFUNCTION_H_
#define TRACKINGFUNCTION_H_

#include <Matrix.hpp>

namespace track
{

// Points to a function that accepts and returns a track::Matrix.
template <class T = double>
using track_fcn = track::Matrix<T> (*)(track::Matrix<T>);

template <class T = double>
class TrackingFunction
{
public:
    TrackingFunction(track_fcn<T> f) : function_impl_(f) {};
    TrackingFunction() {};

    // Stores a user-provided tracking function.
    void register_fcn(track_fcn<T> fcn)
    {
        function_impl_ = fcn;
    }

    // Calls function_impl_(x).
    track::Matrix<T> operator()(track::Matrix<T> x)
    {
        return function_impl_(x);
    }

private:
    track_fcn<T> function_impl_;
};

} // namespace track

#endif // TRACKINGFUNCTION_H_
