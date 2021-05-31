# Track development notes
Planning notes for tracking filter project **track**.

## Overview

### Goals

Create a library of tracking and state estimation filters.

Near term goal: write Kalman, Extended Kalman , JPDA filters.  
Stretch goals: Python interoperability, sensor models, particle filter,
cool examples.

### Filters

The following filters will be implemented in the near term.  
State estimation filters: KF, EKF, UKF  
Tracking filters: JPDA


### Dependencies

[boost](https://www.boost.org) (uses `boost::numeric::ublas`).


---


#### Todo

- [x] Add tests.  
- [x] Write track::Matrix class.    
    - [x] Write matrix inversion method.
    - [ ] Add is-invertible check.
- [ ] Write KF class.  
    - [x] Add `init()`.  
    - [x] Add `predict()`.  
    - [x] Add `update()`.  
    - [ ] Write model-based filter initialization helper functions.
         - [x] 1D const vel
         - [ ] 1D const accel
         - [ ] 2D const vel
         - [ ] 2D const accel
- [ ] Add example to README.
- [ ] Detailed Matrix, KF documentation.
- [ ] Add helper functions for filter init (2d constant acceleration, 3d
    constant acceleration, etc.).
- [ ] Write EKF class.


---


## Resources

[1] Crassidis and Junkins, Optimal Estimation of Dynamic Systems, CRC Press,
2012, pp. 143-149.
