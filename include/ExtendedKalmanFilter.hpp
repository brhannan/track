#ifndef EXTENDED_KALMAN_FILTER_H_
#define EXTENDED_KALMAN_FILTER_H_

#include <EstimationFilter.hpp>
#include <TrackingFunction.hpp>
#include <Matrix.hpp>
#include <filter_init.hpp>

namespace track
{

// EKF class.
// Measurement noise is assumed to be additive.
template <class T = double>
class ExtendedKalmanFilter : public track::EstimationFilter<T>
{
public:
    ExtendedKalmanFilter();
    ExtendedKalmanFilter(std::string motion_model, T dt);

    TrackingFunction<T> state_transition_function;
    TrackingFunction<T> state_transition_function_jacobian;

    TrackingFunction<T> measurement_function;
    TrackingFunction<T> measurement_function_jacobian;

    track::Matrix<T> state;
    track::Matrix<T> measurement_noise;
    track::Matrix<T> process_noise;
    track::Matrix<T> state_covariance;

    void init();
    void validate_properites();

    void predict();
    void update(track::Matrix<T>& y);

protected:
    int M_; // Length of the state vector.
    int N_; // Length of the measurement vector.

private:
    int get_num_states();
    int get_num_measurements();
};

template <class T>
ExtendedKalmanFilter<T>::ExtendedKalmanFilter()
{
}

} // namespace track

#endif // EXTENDED_KALMAN_FILTER_H_
