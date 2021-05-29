#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/storage.hpp>
#include <TrackingFilter.hpp>
#include <Matrix.hpp>

namespace track
{

template <class T = double>
class KalmanFilter : public track::TrackingFilter<T>
{
public:
    KalmanFilter(track::Matrix<T> F, track::Matrix<T> H);
    KalmanFilter(track::Matrix<T> F, track::Matrix<T> H,
        track::Matrix<T> G);
    KalmanFilter();

    track::Matrix<T> state_transition_matrix;
    track::Matrix<T> measurement_matrix;
    track::Matrix<T> control_matrix;
    track::Matrix<T> state;
    track::Matrix<T> state_covariance;
    track::Matrix<T> process_noise;
    track::Matrix<T> measurement_noise;

    void init();
    void predict();
    void update(track::Matrix<T>& y);

protected:
    int M_; // Length of the state vector.
    int N_; // Length of the measurement vector.
    int C_; // Number of control inputs.

private:
    bool has_control_input;
    int get_num_states();
    int get_num_measurements();
    int get_num_controls();
};

template <class T>
KalmanFilter<T>::KalmanFilter(track::Matrix<T> F, track::Matrix<T> H) :
    state_transition_matrix(F), measurement_matrix(H), has_control_input(false)
{
};

template <class T>
KalmanFilter<T>::KalmanFilter(track::Matrix<T> F, track::Matrix<T> H,
    track::Matrix<T> G) :
    state_transition_matrix(F), measurement_matrix(H), control_matrix(G),
    has_control_input(true)
{
};

template <class T>
KalmanFilter<T>::KalmanFilter() : has_control_input(false) {};

// Returns state vector length from state transistion matrix (F). F is a N-by-M
// matrix where N is the number of states.
template <class T>
int KalmanFilter<T>::get_num_states()
{
    return state_transition_matrix.data.size2();
}

// Returns measurement vector length from state transistion matrix (F). F is a
// N-by-M matrix where M is the number of measurements.
template <class T>
int KalmanFilter<T>::get_num_measurements()
{
    return state_transition_matrix.data.size1();
}

// Returns num. control inputs. The number of controls equals the number of
// columns of control_matrix.
template <class T>
int KalmanFilter<T>::get_num_controls()
{
    return control_matrix.data.size2();
}

template <class T>
void KalmanFilter<T>::init()
{
    // Get num. of states, measurements from state transition matrix dimensions.
    M_ = get_num_states();
    N_ = get_num_measurements();
    C_ = get_num_controls();

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

    // Initialize the state vector if it is empty. The default value is a
    // column vector of length M where all values are zeros.
    bool state_uninitialized = this->is_mat_empty(state);
    if (state_uninitialized)
    {
        this->init_column_vector(state, M_);
    }

    // Initialize measurment noise matrix if it is empty. The default value is a
    // N-by-N identity matrix.
    bool meas_noise_uninitialized = this->is_mat_empty(measurement_noise);
    if (meas_noise_uninitialized)
    {
        this->init_from_identity_mat(measurement_noise, M_);
    }
}

// Predicts state, state error covariance.
template <class T>
void KalmanFilter<T>::predict()
{
    // Propagate the state transition matrix.
    track::Matrix<T> x = state_transition_matrix * state;
    // Store the predicted state.
    state = x;
    // Propagate the state covariance.
    state_covariance = 
        state_transition_matrix * state_covariance * \
        state_transition_matrix.transpose() + process_noise;
}

// Updates state, state error covariance.
template <class T>
void KalmanFilter<T>::update(track::Matrix<T>& y)
{
    // TOOD: add update logic here.
}

} // namespace track

#endif // KALMAN_FILTER_H_
