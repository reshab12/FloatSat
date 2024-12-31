/**
 * Test for the KalmanFilter class with 1D projectile motion.
 *
 * @author: Hayk Martirosyan
 * @date: 2014.11.15
 */

#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "kalman.hpp"

int main(int argc, char* argv[]) {

  int n = 2; // Number of states
  int m = 1; // Number of measurements
  int nu = 1; //Number of system inputs
  double R_Gyro = 0.05;
  float u = 1;
  std::vector<float> input = {1.04202710058, 1.10726790452, 1.2913511148, 1.48485250951, 1.72825901034,
      1.74216489744, 2.11672039768, 2.14529225112, 2.16029641405, 2.21269371128,
      2.57709350237, 2.6682215744, 2.51641839428, 2.76034056782, 2.88131780617,
      2.88373786518, 2.9448468727, 2.82866600131, 3.0006601946, 3.12920591669,
      2.858361783, 2.83808170354, 2.68975330958, 2.66533185589, 2.81613499531,
      2.81003612051, 2.88321849354, 2.69789264832, 2.4342229249, 2.23464791825,
      2.30278776224, 2.02069770395, 1.94393985809, 1.82498398739, 1.52526230354,
      1.86967808173, 1.18073207847, 1.10729605087, 0.916168349913, 0.678547664519,
      0.562381751596, 0.355468474885, -0.155607486619, -0.287198661013, -0.602973173813
  };

  double dt = 1.0/50; // Time step

  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd G(n, nu); // System input matrix
  Eigen::MatrixXd C(m,n); // Prediction matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  // Discrete LTI projectile motion, measuring position only
  A << 1, dt*dt, 0, 1;
  G << dt, 1/dt;
  C << 0, 1;

  // Reasonable covariance matrices
  Q << dt*dt * R_Gyro, R_Gyro, R_Gyro, 1/(dt*dt)*R_Gyro;
  R << 10;
  P << 1.0, .0, .0, 1.0;

  std::cout << "A: \n" << A << std::endl;
  std::cout << "G: \n" << G << std::endl;
  std::cout << "C: \n" << C << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "R: \n" << R << std::endl;
  std::cout << "P: \n" << P << std::endl;

  // Construct the filter
  KalmanFilter kf(dt,A, G, C, Q, R, P);

  // List of noisy position measurements (y)
  std::vector<double> measurements = {
      1.04202710058, 1.10726790452, 1.2913511148, 1.48485250951, 1.72825901034,
      1.74216489744, 2.11672039768, 2.14529225112, 2.16029641405, 2.21269371128,
      2.57709350237, 2.6682215744, 2.51641839428, 2.76034056782, 2.88131780617,
      2.88373786518, 2.9448468727, 2.82866600131, 3.0006601946, 3.12920591669,
      2.858361783, 2.83808170354, 2.68975330958, 2.66533185589, 2.81613499531,
      2.81003612051, 2.88321849354, 2.69789264832, 2.4342229249, 2.23464791825,
      2.30278776224, 2.02069770395, 1.94393985809, 1.82498398739, 1.52526230354,
      1.86967808173, 1.18073207847, 1.10729605087, 0.916168349913, 0.678547664519,
      0.562381751596, 0.355468474885, -0.155607486619, -0.287198661013, -0.602973173813
  };

  // Best guess of initial states
  Eigen::VectorXd x0(n);
  double t = 0;
  x0 << 0,0;
  kf.init(t, x0);

  // Feed measurements into filter, output estimated states

  Eigen::VectorXd y(m);
  std::cout << "t = " << t << ", " << "x_hat[0]: " << kf.state().transpose() << std::endl;
  for(int i = 0; i < measurements.size()/2 - 1; i = i+2) {
    t += dt;
    y << measurements[i];
    u = input[i];
    kf.update(y,u);
    std::cout << "t = " << t << ", " << "y[" << i << "] = " << y.transpose()
        << ", x_hat[" << i << "] = " << kf.state().transpose() << std::endl;
  }

  return 0;
}
