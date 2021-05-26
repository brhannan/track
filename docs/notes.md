# Track development notes
Planning notes for tracking filter project **track**.

## Overview

### Goal

Create a library of tracking and state estimation filters.

### Strategy

Each filter is a class.

Each state estimation filter has a number of matrices (state transition matrix,
measurement matrix, etc.).
Each matrix inherits from `TrackingMatrix`. Rationale: allows type/dimensions
checking and default-value initialization.

### Filters

The following filters will be implemented in the near term.  
State estimation filters: KF, EKF, UKF  
Tracking filters: JPDA


### Dependencies

[boost](https://www.boost.org) (uses `boost::numeric::ublas`).


---


## Tracking Kalman filter

### Parameters

#### Public

* `state_transition_matrix` F_k. An N-by-M matrix where N is the number of
elements of the measurement vector and M is the number of states. Default:
* `measurement_matrix` H_k. An N-by-M matrix.
* `state` vector x_k. Scalar or vector (length = M).
* `control_matrix` G_k (optional). A M-by-C matrix where C is the number of
control inputs.
* `process_noise` v_k. May be a positive scalar or a M-by-M matrix. Positive-definite.
* `measurement_noise` w_k. A positive scalar or a positive-definite, real N-by-N
matrix.
* `state`. State vector.
* `state_covariance`
* `process_noise`
* `dt`

#### Protected

* `M_`. Number of elements of the measurement vector. Positive nonzero integer.
* `N_`. Length of the state vector. Positive nonzero integer.
* `C_`. Number of control inputs. An integer >= 0. If a control model is not provided,
`C_` equals 0.
* `state_covariance` P_k. A positive scalar or a M-by-M matrix. Positive-definite.


### Methods

#### Constructor

`KF::KF()`

#### Predict

The Kalman filter's predict step calculates the state and state estimation error
covariance.

```
template <class T>
T KF::predict(const T& dt)
```
Executes KF predict step for time interval `dt`.

```
T KF::predict(const T& dt, const std::vector<T>& u)
```
Applies control input `u`. Use this syntax when `ControlPolicy` is
`ControlInputEnabled`.


#### Update

Update state and state estimation error covariance.

```
template <class T>
KF::update(const std::vector<T>& y)
```


#### Initialize

`KF::init()`

#### Todo

- [ ] Add tests.  
- [x] Write tracking matrix base class.  
- [x] Write state transition matrix (F) class.  
- [ ] Write measurement model matrix (H) class.  
- [ ] Write process noise covariance matrix (Q) class.  
- [ ] Write control input matrix (C) class.  
- [ ] Write measurement (z) class.  
- [ ] Write state error covariance (P) class.  
- [ ] Write KF class.  

<!-- ### Policy classes

#### Control policy

`ControlPolicy` may have type `ControlInputEnabled` or `ControlInputDisabled`.

`MotionModelPolicy` may have type
`<1|2|3>DConstant<Velocity|Acceleration>Motion`
or `CustomMotion`. -->


---


## Tracking filter base class

Virtual methods: `predict()`, `update()`, `init()`.

Member variables: `state`, `state_covariance`, `process_noise`,
`measurement_noise`, `state_transition_matrix`, `control_matrix`,
`measurement_matrix`, `dt`.


---


## Resources

[1] Crassidis and Junkins, Optimal Estimation of Dynamic Systems, CRC Press,
2012, pp. 143-149.
