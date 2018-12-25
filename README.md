# Extended Kalman Filter Project
With start code from [Udacity EKF project.](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project)

A useful tutorial can be found [here.](https://towardsdatascience.com/extended-kalman-filter-43e52b16757d)
* Approximate non-linear function *g(x)* by first order Taylor expansion about g(*mu*), where *mu* is the mean of *x*.

## Data
Data are generated by a simulator. The communication between the simulator and my program is done by uWebSocketIO
which is installed using `./install-linux.sh`  
* Lidar
* Radar
    * [rho, phi, rho_dot] in Polar coordinate system 
    * Non-linear


## Update step
### for Radar
* Mapping function **h(x')**:
    * Because we are predicting(*x'*) in Cartesian coordinates but our measurements(*z*) from **Radar** is in Polar coordinates;  
    * ![Mapping function](Docs/pics/h.png)
* Jacobian matrix **H_j**
    * first order derivatives in Taylor expansion
    * ![Jacobian matrix H_j](Docs/pics/Hj.png)