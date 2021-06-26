#ifndef TRACKINGFUNCTION_H_
#define TRACKINGFUNCTION_H_

// #include <AbstractTrackingFunction.hpp>
#include <Matrix.hpp>

namespace track
{

// Points to a user-defined function that implements the tracking function.
template <class T = double>
using track_fcn = track::Matrix<T> (*)(track::Matrix<T>, track::Matrix<T>, T);

template <class T = double>
class TrackingFunction // : public track::AbstractTrackingFunction<T>
{
public:
    TrackingFunction(track_fcn<T> f)
    : user_provided_fcn_(f), zero_mat(Matrix<T>("zeros",1,1))
    {
    };

    TrackingFunction() : zero_mat(Matrix<T>("zeros",1,1)) {};

    // Registers a user-provided tracking function.
    void register_fcn(track_fcn<T> fcn)
    {
        user_provided_fcn_ = fcn;
    }

    // Calls calls_user_provided_fcn_.
    track::Matrix<T> operator()(track::Matrix<T> x, T dt = 1)
    {
        return  user_provided_fcn_(x,zero_mat,dt);
    }

    // Calls calls_user_provided_fcn_. This syntax is provided to support
    // tracking functions that require two matrix input arguents.
    track::Matrix<T> operator()(track::Matrix<T> m1, track::Matrix<T> m2,
        T dt = 1)
    {
        return  user_provided_fcn_(m1,m2,dt);
    }

    // Calls calls_user_provided_fcn_.
    track::Matrix<T> step(track::Matrix<T> x, T dt = 1)
    {
        return user_provided_fcn_(x,zero_mat,dt);
    }

    // Calls calls_user_provided_fcn_. This syntax is provided to support
    // tracking functions that require two matrix input arguents.
    track::Matrix<T> step(track::Matrix<T> m1, track::Matrix<T> m2, T dt = 1)
    {
        return user_provided_fcn_(m1,m2,dt);
    }

private:
    track_fcn<T> user_provided_fcn_;
    // zero_mat is a "dummy" matrix. If step(x) or step(x,t) is called (if
    // only one matrix parameter is provided), zero_mat is passed as the second
    // matrix parameter to user_provided_fcn_. zero_mat is stored as a member
    // of TrackingFunction to improve performance.
    Matrix<T> zero_mat;
};

} // namespace track

#endif // TRACKINGFUNCTION_H_
