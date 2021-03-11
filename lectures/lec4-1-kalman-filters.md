# Lecture 4-1 Kalman Filters

## I. Gaussian Function

Gaussian functions are often used to represent the probability density function of a normally distributed random variable with the **expected value (mean)** `mu` and **variance** `sigma^2`.

<img src="media/gaussian-distribution-equation.png" width="300" height="100" />

- Evaluations
    * the max value of the Gaussian function is obtained when `x = mu`
    * different mean `mu` shifts the distribution toward `+` or `-` of the x-axis
    * larger the variance `sigma^2`, wider the spread of distribution

<img src="media/gaussian-distribution.png" width="600" height="400" />

## II. Kalman Filter

- Kalman filter represents our distributions by Gaussian and iterates on two main cycles
    * First cycle: **measurement update**
        + requires a product/multiplication
        + uses Bayes rule
    * Second cycle: **motion update** (prediction)
        + involves a convolution
        + uses total probability

#### Measurement update

- Shift of the mean
    * the new belief will be *somewhere between* the previous belief and the new measurement
- Predict the peak
    * the new belief will be *more certain* than either the previous belief or the new measurement
    * because more measurements means greater certainty

<img src="media/gaussian-shift-after-new-measurement.png" width="400" height="300" />

The mean and variance of the new Gaussian can be calculated by the following equations.

<img src="media/gaussian-measurement-update.png" width="700" height="300" />

#### Motion update (Prediction)

The Gaussian parameters for motion update are much simpler than measurement update, just adding the two means and two variance to yield the new ones.

<img src="media/gaussian-motion-update.png" width="500" height="300" />

#### Python implementation

```python
# iteratively update and predict based on the location measurements and inferred motions shown below. 

def update(mean1, var1, mean2, var2):
    new_mean = float(var2 * mean1 + var1 * mean2) / (var1 + var2)
    new_var = 1./(1./var1 + 1./var2)
    return [new_mean, new_var]

def predict(mean1, var1, mean2, var2):
    new_mean = mean1 + mean2
    new_var = var1 + var2
    return [new_mean, new_var]

measurements = [5., 6., 7., 9., 10.]
motion = [1., 1., 2., 1., 1.]
measurement_sig = 4.
motion_sig = 2.
mu = 0.
sig = 10000.

for n in range(len(measurements)):
    [mu, sig] = update(mu, sig, measurements[n], measurement_sig)
    print 'update: ', [mu, sig]
    [mu, sig] = predict(mu, sig, motion[n], motion_sig)
    print 'predict: ', [mu, sig]
```

## III. Kalman Filter Design

High Dimensional Gaussian or Multivariate Gaussian features a *k*-dimensional **mean vector**, and a *k x k* **covariance matrix**.

<img src="media/multivariate-gaussian.png" width="400" height="300" />

For a Kalman filter for vehicle, the two dimensions to use are *location* and *velocity*.

#### KF Design

`x`: location estimation
`P`: uncertainty covariance
`F`: state transition matrix
`v`: motion vector
`Z`: measurement
`H`: measurement function
`y`: error
`S`: matrix projecting the system uncertainty into the measurement space
`R`: measurement noise matrix
`K`: Kalman filter gain, combining the uncertainty of where we think we are `P` and uncertainty of sensor measurement `R`. If `P > R`, KF will give more weight to sensor measurement `z`. If `P < R`, more weight will be put on predicted `x`.
`I`: identity matrix

<img src="media/kalman-filter-design.png" width="900" height="250" />

```python
from math import *

class matrix:
    '''
    implements basic operations of a matrix class
    '''
    def __init__(self, value):
        self.value = value
        self.dimx = len(value)
        self.dimy = len(value[0])
        if value == [[]]:
            self.dimx = 0
    
    def zero(self, dimx, dimy):
        # check if valid dimensions
        if dimx < 1 or dimy < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dimx
            self.dimy = dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]
    
    def identity(self, dim):
        # check if valid dimension
        if dim < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dim
            self.dimy = dim
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1
    
    def show(self):
        for i in range(self.dimx):
            print(self.value[i])
        print(' ')
    
    def __add__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to add"
        else:
            # add if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res
    
    def __sub__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to subtract"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res
    
    def __mul__(self, other):
        # check if correct dimensions
        if self.dimy != other.dimx:
            raise ValueError, "Matrices must be m*n and n*p to multiply"
        else:
            # multiply if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
            return res
    
    def transpose(self):
        # compute transpose
        res = matrix([[]])
        res.zero(self.dimy, self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res
    
    # Thanks to Ernesto P. Adorio for use of Cholesky and CholeskyInverse functions
    
    def Cholesky(self, ztol=1.0e-5):
        # Computes the upper triangular Cholesky factorization of
        # a positive definite matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)
        
        for i in range(self.dimx):
            S = sum([(res.value[k][i])**2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol:
                res.value[i][i] = 0.0
            else:
                if d < 0.0:
                    raise ValueError, "Matrix not positive-definite"
                res.value[i][i] = sqrt(d)
            for j in range(i+1, self.dimx):
                S = sum([res.value[k][i] * res.value[k][j] for k in range(self.dimx)])
                if abs(S) < ztol:
                    S = 0.0
                try:
                   res.value[i][j] = (self.value[i][j] - S)/res.value[i][i]
                except:
                   raise ValueError, "Zero diagonal"
        return res
    
    def CholeskyInverse(self):
        # Computes inverse of matrix given its Cholesky upper Triangular
        # decomposition of matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)
        
        # Backward step for inverse.
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k]*res.value[j][k] for k in range(j+1, self.dimx)])
            res.value[j][j] = 1.0/tjj**2 - S/tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j] = -sum([self.value[i][k]*res.value[k][j] for k in range(i+1, self.dimx)])/self.value[i][i]
        return res
    
    def inverse(self):
        aux = self.Cholesky()
        res = aux.CholeskyInverse()
        return res
    
    def __repr__(self):
        return repr(self.value)

# KF implementation
def kalman_filter(x, P):
    for n in range(len(measurements)):
        # measurement update
        Z = matrix([[measurements[n]]])
        y = Z - (H * x)
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S.inverse()
        x = x + (K * y)
        P = (I - (K * H)) * P

        # prediction
        x = (F * x) + u
        P = F * P * F.transpose()
    
    # return location estimation, uncertainty covariance
    return x, P

# Test Below
measurements = [1, 2, 3]

x = matrix([[0.], [0.]]) # initial state (location and velocity)
P = matrix([[1000., 0.], [0., 1000.]]) # initial uncertainty
u = matrix([[0.], [0.]]) # external motion
F = matrix([[1., 1.], [0, 1.]]) # next state function
H = matrix([[1., 0.]]) # measurement function
R = matrix([[1.]]) # measurement uncertainty
I = matrix([[1., 0.], [0., 1.]]) # identity matrix

print(kalman_filter(x, P))
# output should be:
# x: [[3.9996664447958645], [0.9999998335552873]]
# P: [[2.3318904241194827, 0.9991676099921091], [0.9991676099921067, 0.49950058263974184]]
```

## IV. Equation Cheatsheet

[Sensor Fusion EKF Reference.pdf](../Kalman_Filters/sensor-fusion-ekf-reference.pdf)