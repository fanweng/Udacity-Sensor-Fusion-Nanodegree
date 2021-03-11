/**
 * Write a function 'filter()' that implements a multi-
 *   dimensional Kalman Filter for the example given
 */

#include <iostream>
#include <vector>
#include "Dense"

using std::cout;
using std::endl;
using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;

// Kalman Filter variables
VectorXd x;	// object state
MatrixXd P;	// object covariance matrix
VectorXd u;	// external motion
MatrixXd F; // state transition matrix
MatrixXd H;	// measurement matrix
MatrixXd R;	// measurement covariance matrix
MatrixXd I; // Identity matrix
MatrixXd Q;	// process covariance matrix

vector<VectorXd> measurements;
void filter(VectorXd &x, MatrixXd &P);


int main() {
    /**
     * Code used as example to work with Eigen matrices
     */
    // design the KF with 1D motion
    x = VectorXd(2);
    x << 0, 0;

    P = MatrixXd(2, 2);
    P << 1000, 0, 0, 1000;

    u = VectorXd(2);
    u << 0, 0;

    F = MatrixXd(2, 2);
    F << 1, 1, 0, 1;

    H = MatrixXd(1, 2);
    H << 1, 0;

    R = MatrixXd(1, 1);
    R << 1;

    I = MatrixXd::Identity(2, 2);

    Q = MatrixXd(2, 2);
    Q << 0, 0, 0, 0;

    // create a list of measurements
    VectorXd single_meas(1);
    single_meas << 1;
    measurements.push_back(single_meas);
    single_meas << 2;
    measurements.push_back(single_meas);
    single_meas << 3;
    measurements.push_back(single_meas);

    // call Kalman filter algorithm
    filter(x, P);

    return 0;
}


void filter(VectorXd &x, MatrixXd &P) {

    for (unsigned int n = 0; n < measurements.size(); ++n) {

        VectorXd z = measurements[n];

        // KF Measurement update step
        VectorXd y = z - H * x;
        MatrixXd S = H * P * H.transpose() + R;
        MatrixXd K = P * H.transpose() * S.inverse();

        // new state
        x = x + K * y;
        P = (I - K * H) * P;

        // KF Prediction step
        x = F * x + u;
        P = F * P * F.transpose() + Q;

        cout << "x=" << endl << x << endl;
        cout << "P=" << endl << P << endl;
    }
}