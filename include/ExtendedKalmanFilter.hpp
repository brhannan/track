#ifndef EXTENDED_KALMAN_FILTER_H_
#define EXTENDED_KALMAN_FILTER_H_

#include <EstimationFilter.hpp>
#include <TrackingFunction.hpp>
#include <Matrix.hpp>
#include <filter_init.hpp>

// TODO:
//  Add policy that allows the user to disable dimensions checking on
//  tracking function IO.

namespace track
{

// EKF class.
// Measurement noise is assumed to be additive.
template <class T = double>
class ExtendedKalmanFilter : public track::EstimationFilter<T>
{
public:
    ExtendedKalmanFilter(int num_states, int num_meas);
    ExtendedKalmanFilter(int num_states, int num_meas,
        TrackingFunction<T> f, TrackingFunction<T> fj, TrackingFunction<T> h,
        TrackingFunction<T> hj);
    ExtendedKalmanFilter(std::string motion_model, T dt);
    ExtendedKalmanFilter();

    TrackingFunction<T>* state_transition_function;
    TrackingFunction<T>* state_transition_function_jacobian;

    TrackingFunction<T>* measurement_function;
    TrackingFunction<T>* measurement_function_jacobian;

    track::Matrix<T> state;
    track::Matrix<T> measurement_noise;
    track::Matrix<T> process_noise;
    track::Matrix<T> state_covariance;

    void init(); // TODO: move to base class ***********************************
    void validate_properites(); // TODO: move to base class ********************

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
ExtendedKalmanFilter<T>::ExtendedKalmanFilter(int num_states, int num_meas)
    : M_(num_states), N_(num_meas)
{
}


template <class T>
ExtendedKalmanFilter<T>::ExtendedKalmanFilter(
    int num_states, int num_meas,
    TrackingFunction<T> f, TrackingFunction<T> fj, TrackingFunction<T> h,
    TrackingFunction<T> hj)
    : M_(num_states), N_(num_meas),
    state_transition_function(&f),
    state_transition_function_jacobian(fj),
    measurement_function(&h),
    measurement_function_jacobian(&hj)
{
}


template <class T>
ExtendedKalmanFilter<T>::ExtendedKalmanFilter()
{
    // TODO: Initialize from motion model helper.
}


template <class T>
void ExtendedKalmanFilter<T>::init()
{
    // Initialize state covariance matrix if it is empty. The default value is a
    // M-by-M identity matrix.
    bool state_cov_uninitialized = this->is_mat_empty(state_covariance);
    if (state_cov_uninitialized)
    {
        this->init_from_identity_mat(state_covariance, M_);
    }

    // Initialize process noise matrix if it is empty. The default value is a
    // M-by-M identity matrix.
    bool proc_noise_uninitialized = this->is_mat_empty(process_noise);
    if (proc_noise_uninitialized)
    {
        this->init_from_identity_mat(process_noise, M_);
    }

    // Initialize measurment noise matrix if it is empty. The default value is a
    // N-by-N identity matrix.
    bool meas_noise_uninitialized = this->is_mat_empty(measurement_noise);
    if (meas_noise_uninitialized)
    {
        this->init_from_identity_mat(measurement_noise, M_);
    }

    // Initialize the state vector. The default value is a column vector of
    // length M where all values are zeros.
    bool state_uninitialized = this->is_mat_empty(state);
    if (state_uninitialized)
    {
        this->init_column_vector(state, M_);
    }

    // Validate filter properties.
    validate_properites();
}


template <class T>
void ExtendedKalmanFilter<T>::validate_properites()
{
    if (M_ <= 0)
    {
        throw std::invalid_argument("Num. measurements must be > 0.");
    }
    if (N_ <= 0)
    {
        throw std::invalid_argument("Num. states must be > 0.");
    }

    int proc_noise_num_rows = process_noise.num_rows();
    int proc_noise_num_cols = process_noise.num_cols();
    if (proc_noise_num_rows != M_ | proc_noise_num_cols != M_)
    {
        this->get_invalid_dims_err_msg("process_noise",
            M_, M_, proc_noise_num_rows, proc_noise_num_cols);
    }

    int meas_noise_num_rows = measurement_noise.num_rows();
    int meas_noise_num_cols = measurement_noise.num_cols();
    if (meas_noise_num_rows != N_ | meas_noise_num_cols != N_)
    {
        this->get_invalid_dims_err_msg("process_noise",
            N_, N_, meas_noise_num_rows, meas_noise_num_cols);
    }
}


template <class T>
int ExtendedKalmanFilter<T>::get_num_states()
{
    return M_;
}


template <class T>
int ExtendedKalmanFilter<T>::get_num_measurements()
{
    return N_;
}


template <class T>
void ExtendedKalmanFilter<T>::predict()
{
    state = measurement_function->step(state);
}


template <class T>
void ExtendedKalmanFilter<T>::update(track::Matrix<T>& y)
{
}

} // namespace track

#endif // EXTENDED_KALMAN_FILTER_H_
