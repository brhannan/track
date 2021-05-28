#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/storage.hpp>
#include <stdexcept>
#include <Matrix.hpp>

namespace track
{

template <class T = double>
class KalmanFilter
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
    // predict();
    // update();

protected:
    int M_; // Length of the state vector.
    int N_; // Length of the measurement vector.
    int C_; // Number of control inputs.

private:
    bool has_control_input;
    int get_num_states();
    int get_num_measurements();
    int get_num_controls();
    bool is_mat_empty(track::Matrix<T>& mat);
    void initialize_from_identity_matrix(track::Matrix<T>& mat, int L, T v = 1);
    void initialize_column_vector(track::Matrix<T>& vec, int L, T v = 0);
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

// Returns true if mat is empty.
template <class T>
bool KalmanFilter<T>::is_mat_empty(track::Matrix<T>& mat)
{
    return (mat.data.size1()==0) && (mat.data.size2()==0);
}

// Initialize a square matrix by scaling an identity matrix of dimension L. The
// matrix is scaled by scalar v. The result is returned to mat.
template <class T>
void KalmanFilter<T>::initialize_from_identity_matrix(track::Matrix<T>& mat,
    int L, T v)
{
    boost::numeric::ublas::identity_matrix<T> ident(L);
    mat.data = v * ident;
}

// Initialize a column vector by scaling a vector of dimension L. All values
// are set to value v (default 0).
template <class T>
void KalmanFilter<T>::initialize_column_vector(track::Matrix<T>& vec, int L, T v)
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


template <class T>
void KalmanFilter<T>::init()
{
    // Get num. of states, measurements from state transition matrix dimensions.
    M_ = get_num_states();
    N_ = get_num_measurements();
    C_ = get_num_controls();

    // Initialize state covariance matrix if it is empty. The default value is a
    // M-by-M identity matrix.
    bool state_cov_uninitialized = is_mat_empty(state_covariance);
    if (state_cov_uninitialized)
    {
        initialize_from_identity_matrix(state_covariance, M_);
    }

    // Initialize process noise matrix if it is empty. The default value is a
    // M-by-M identity matrix.
    bool proc_noise_uninitialized = is_mat_empty(process_noise);
    if (proc_noise_uninitialized)
    {
        initialize_from_identity_matrix(process_noise, M_);
    }

    // Initialize the state vector if it is empty. The default value is a
    // column vector of length M where all values are zeros.
    bool state_uninitialized = is_mat_empty(state);
    if (state_uninitialized)
    {
        initialize_column_vector(state, M_);
    }

    // Initialize measurment noise matrix if it is empty. The default value is a
    // N-by-N identity matrix.
    bool meas_noise_uninitialized = is_mat_empty(measurement_noise);
    if (meas_noise_uninitialized)
    {
        initialize_from_identity_matrix(measurement_noise, M_);
    }
}

} // namespace track

#endif // KALMAN_FILTER_H_
