// Filter initialization utilities.

#ifndef FILT_INIIT_H_
#define FILT_INIIT_H_

#include <Matrix.hpp>

// Stores Kalman filter initialization method outputs.
template <class T>
struct KFParams
{
    track::Matrix<T> state_transition_matrix;
    track::Matrix<T> measurement_matrix;
    track::Matrix<T> process_noise;
    track::Matrix<T> measurement_noise;
    int M; // Num. states.
    int N; // Num. measurements.
};

#endif // FILT_INIIT_H_
