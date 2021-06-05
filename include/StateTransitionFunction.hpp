// StateTransitionFunction class. Implements EKF state transition function.

#ifndef STATETRANSITIONFUNCTION_H_
#define STATETRANSITIONFUNCTION_H_

#include <Matrix.hpp>

namespace track
{

// Points to a function that implements a state transition function.
template <class T = double>
using stf = track::Matrix<T> (*)(track::Matrix<T>);

template <class T = double>
class StateTransitionFunction
{
public:
    StateTransitionFunction() {};

    // Stores a user-provided state transition function.
    void registerStateTransitionFunction(stf<T> fcn)
    {
        stateTransitionFcn_ = fcn;
    }

    // Calls stateTransitionFcn_(x).
    track::Matrix<T> operator()(track::Matrix<T> x)
    {
        return stateTransitionFcn_(x);
    }

private:
    stf<T> stateTransitionFcn_;
};

} // namespace track

#endif // STATETRANSITIONFUNCTION_H_
