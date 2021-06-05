#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/storage.hpp>
#include <TrackingFilter.hpp>
#include <Matrix.hpp>
#include <filter_init.hpp>

namespace track
{

template <class T = double>
class KalmanFilter : public track::TrackingFilter<T>
{
public:
    KalmanFilter(track::Matrix<T> F, track::Matrix<T> H);
    KalmanFilter(track::Matrix<T> F, track::Matrix<T> H,
        track::Matrix<T> G);
    KalmanFilter(std::string motion_model, T dt);
    KalmanFilter();

    track::Matrix<T> state_transition_matrix;
    track::Matrix<T> measurement_matrix;
    track::Matrix<T> control_matrix;
    track::Matrix<T> state;
    track::Matrix<T> state_covariance;
    track::Matrix<T> process_noise;
    track::Matrix<T> measurement_noise;

    void init();
    void validate_properites();

    void predict(const track::Matrix<T> &u = track::Matrix<T>("zeros",2,1));
    void update(track::Matrix<T>& y);

    static KFParams<T> get_model_1d_const_vel(T dt);
    static KFParams<T> get_model_2d_const_vel(T dt);
    // static KFParams<T> get_model_3d_const_vel(T dt);

    // static KFParams<T> get_model_1d_const_accel(T dt);
    // static KFParams<T> get_model_2d_const_accel(T dt);
    // static KFParams<T> get_model_3d_const_accel(T dt);

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
}

template <class T>
KalmanFilter<T>::KalmanFilter(track::Matrix<T> F, track::Matrix<T> H,
    track::Matrix<T> G) :
    state_transition_matrix(F), measurement_matrix(H), control_matrix(G),
    has_control_input(true)
{
}

// Constructs a `KalmanFilter` using motion profile `motion_profile` and
// period  `dt` (sec.).
// `motion_profile` may be one of the following:
//  "1d_const_vel", "2d_const_vel", "3d_const_vel",
//  "1d_const_accel", "2d_const_accel", "3d_const_accel".
template <class T>
KalmanFilter<T>::KalmanFilter(std::string motion_model, T dt)
    : has_control_input(false)
{
    if (motion_model == "1d_const_vel")
    {
        KFParams<T> p = get_model_1d_const_vel(dt);
        state_transition_matrix = p.state_transition_matrix;
        measurement_matrix = p.measurement_matrix;
        process_noise = p.process_noise;
        measurement_noise = p.measurement_noise;
    }
    else if (motion_model == "2d_const_vel")
    {
        KFParams<T> p = get_model_2d_const_vel(dt);
        state_transition_matrix = p.state_transition_matrix;
        measurement_matrix = p.measurement_matrix;
        process_noise = p.process_noise;
        measurement_noise = p.measurement_noise;
    }
    // else if (motion_model == "3d_const_vel")
    // {
    //     KFParams<T> p = get_model_3d_const_vel(dt);
    //     state_transition_matrix = p.state_transition_matrix;
    //     measurement_matrix = p.measurement_matrix;
    //     process_noise = p.process_noise;
    //     measurement_noise = p.measurement_noise;
    // }
    // else if (motion_model == "1d_const_accel")
    // {
    //     KFParams<T> p = get_model_1d_const_accel(dt);
    //     state_transition_matrix = p.state_transition_matrix;
    //     measurement_matrix = p.measurement_matrix;
    //     process_noise = p.process_noise;
    //     measurement_noise = p.measurement_noise;
    // }
    // else if (motion_model == "2d_const_accel")
    // {
    //     KFParams<T> p = get_model_2d_const_accel(dt);
    //     state_transition_matrix = p.state_transition_matrix;
    //     measurement_matrix = p.measurement_matrix;
    //     process_noise = p.process_noise;
    //     measurement_noise = p.measurement_noise;
    // }
    // else if (motion_model == "3d_const_accel")
    // {
    //     KFParams<T> p = get_model_3d_const_accel(dt);
    //     state_transition_matrix = p.state_transition_matrix;
    //     measurement_matrix = p.measurement_matrix;
    //     process_noise = p.process_noise;
    //     measurement_noise = p.measurement_noise;
    // }
    else
    {
        throw std::invalid_argument("Expected input motion_model to equal one "
            "of the following: \"1d_const_vel\", \"2d_const_vel\", "
            "\"3d_const_vel\", \"1d_const_accel\", \"2d_const_accel\", "
            "\"3d_const_accel\" .");
    }
}

template <class T>
KalmanFilter<T>::KalmanFilter() : has_control_input(false) {};

// Returns state vector length from state transistion matrix (F). F is a N-by-M
// matrix where N is the number of states.
template <class T>
int KalmanFilter<T>::get_num_states()
{
    return state_transition_matrix.data.size1();
}

// Returns measurement vector length from state transistion matrix (F). F is a
// N-by-M matrix where M is the number of measurements.
template <class T>
int KalmanFilter<T>::get_num_measurements()
{
    return measurement_matrix.data.size1();
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

    // Validate filter properties.
    validate_properites();
}

// Validates filter properties.
template <class T>
void KalmanFilter<T>::validate_properites()
{
    // State transition matrix is expected to be M,s-by-M.
    int stm_n_rows = state_transition_matrix.num_rows();
    int stm_n_cols = state_transition_matrix.num_cols();
    if (stm_n_rows!=M_ | stm_n_cols!=M_)
    {
        throw std::invalid_argument(
            this->get_invalid_dims_err_msg("state_transition_matrix",
                M_, M_, stm_n_rows, stm_n_cols)
        );
    }

    // Measurement matrix is expected to be N-by-M.
    int h_n_rows = measurement_matrix.num_rows();
    int h_n_cols = measurement_matrix.num_cols();
    if (h_n_rows!=N_ | stm_n_cols!=M_)
    {
        throw std::invalid_argument(
            this->get_invalid_dims_err_msg("measurement_matrix",
                N_, M_, h_n_rows, h_n_cols)
        );
    }

    // If configured to accept control inputs, ensure that control_matrix is a
    // M-by-C matrix where M is the number of states and C is the number of
    // controls.
    if (has_control_input)
    {
        int ctrl_n_rows = control_matrix.num_rows();
        int ctrl_n_cols = control_matrix.num_cols();
        if (ctrl_n_rows!=M_ | ctrl_n_cols!=C_)
        {
            throw std::invalid_argument(
                this->get_invalid_dims_err_msg("control_matrix",
                    M_, C_, ctrl_n_rows, ctrl_n_cols)
                );
        }
    }

    // TODO:
    //  Add more dims validation here.
}

// Predicts state, state error covariance. Input is control vector.
template <class T>
void KalmanFilter<T>::predict(const track::Matrix<T>& u)
{
    // Propagate the state transition matrix.
    track::Matrix<T> x = state_transition_matrix * state;

    if (has_control_input)
    {
        int u_num_rows = u.num_rows();
        int u_num_cols = u.num_cols();
        if (u_num_rows != C_)
        {
            throw std::invalid_argument(
                std::string("Expected control vector to have ") +
                std::to_string(C_) +
                std::string(" elements.")
            );
        }
        if (u_num_cols != 1)
        {
            throw std::invalid_argument(
                std::string("Expected control vector to have 1 column.")
            );
        }
        x = x + control_matrix * u;
    }
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
    // Calculate gain K.
    track::Matrix<T> K = ( state_covariance * measurement_matrix.transpose() ) *
        ( measurement_matrix * state_covariance * measurement_matrix.transpose()
        + measurement_noise ).inverse();
    // Updade the state vector.
    state = state + K*(y - measurement_matrix*state);
    // Update the state covariance matrix.
    state_covariance = state_covariance -
        K * measurement_matrix * state_covariance;
}

// Gets 1D constant velocity motion model parameters. Outputs are returned in
// a KFParams struct.
//
// Input dt is a duration in seconds. It is used to create the state transition
// matrix.
//
// Outputs are:
//      M (num. measurments)        2
//      N (num. states)             2
//      state_transistion_matrix    [ 1 dt; 0 1 ]
//      measurement_matrix          [ 1, 0; 0, 1 ]
//      process_noise               M-by-M identity matrix.
//      measurement_noise           N-by-N identity matrix.
//
// The state vector is a column vector containing the following elements:
//      [ x; v_x ; y; v_y ]
//
// The semicolons above indicate the end of a row.
template <class T>
KFParams<T> KalmanFilter<T>::get_model_1d_const_vel(T dt)
{
    int M = 2;
    int N = 2;
    std::vector<T> stm_vals = {1, dt, 0, 1};
    track::Matrix<T> F(stm_vals,M,M);
    std::vector<T> meas_vals = {1, 0, 0, 1};
    track::Matrix<T> H(meas_vals,N,M);
    track::Matrix<T> Q("identity",M,M);
    track::Matrix<T> V("identity",N,N);
    // Return filter params in KFParams struct.
    KFParams<T> out;
    out.state_transition_matrix = F;
    out.measurement_matrix = H;
    out.process_noise = Q;
    out.measurement_noise = V;
    out.M = M;
    out.N = N;
    return out;
}

// Gets 2D constant velocity motion profile parameters. Outputs are returned in
// a KFParams struct.
//
// Input dt is a duration in seconds. It is used to create the state transition
// matrix.
//
// Outputs are:
//      M (num. measurments)        2
//      N (num. states)             4
//      state_transistion_matrix    [ 1 dt; 0 1 ]
//      measurement_matrix          [ 1, 0, 0, 0;  0, 1, 0, 0 ]
//      process_noise               M-by-M identity matrix.
//      measurement_noise           N-by-N identity matrix.
//
// The state vector is a column vector containing the following elements:
//      [ x; v_x; y; v_y ]
//
// The semicolons above indicate the end of a row.
template <class T>
KFParams<T> KalmanFilter<T>::get_model_2d_const_vel(T dt)
{
    int M = 4;
    int N = 2;
    std::vector<T> stm_vals = {
        1, dt, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, dt,
        0, 0, 0, 1
    };
    track::Matrix<T> F(stm_vals,M,M);
    std::vector<T> meas_vals = {
        1, 0, 0, 0,
        0, 0, 1, 0
    };
    track::Matrix<T> H(meas_vals,N,M);
    track::Matrix<T> Q("identity",M,M);
    track::Matrix<T> V("identity",N,N);
    // Return filter params in KFParams struct.
    KFParams<T> out;
    out.state_transition_matrix = F;
    out.measurement_matrix = H;
    out.process_noise = Q;
    out.measurement_noise = V;
    out.M = M;
    out.N = N;
    return out;
}

// TODO: Add more default motion profiles here.

} // namespace track

#endif // KALMAN_FILTER_H_
