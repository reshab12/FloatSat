#include "kalman_thread.hpp"



    KalmanThread::KalmanThread() : Subscriber(topic_imu_data, "kalmanThread") {}

    uint32_t KalmanThread::put(const uint32_t topicId, const size_t len, void *data, [[gnu::unused]] const NetMsgInfo &netMsgInfo) {

        inputMsgBuffer.put(*(imu_data *) data);
        this->resume();                         // not to publish from interrupt, call a thread to do it
        return 1;
    }

    void KalmanThread::init(){

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

        /*std::cout << "A: \n" << A << std::endl;
        std::cout << "G: \n" << G << std::endl;
        std::cout << "C: \n" << C << std::endl;
        std::cout << "Q: \n" << Q << std::endl;
        std::cout << "R: \n" << R << std::endl;
        std::cout << "P: \n" << P << std::endl;*/

        // Construct the filter
        KalmanFilter kf(dt,A, G, C, Q, R, P);

        // Best guess of initial states
        Eigen::VectorXd x0(n);

        x0 << 0,0;
        kf.init(t, x0);

        // Feed measurements into filter, output estimated states

        kalmanFilter = kf;

    }

    void KalmanThread::run() {
        imu_data cnt;
        while (1) {
            suspendCallerUntil(); // the subscriber shall reactivate it
            inputMsgBuffer.get(cnt);

            Attitude attitude;

            float mag[3];
            mag[0] = cnt.mx;
            mag[1] = cnt.my;
            mag[2] = cnt.mz;

            calcHeadingMagneto(&attitude, mag, 0, 0);

            Eigen::VectorXd y(m);
            //std::cout << "t = " << t << ", " << "x_hat[0]: " << kf.state().transpose() << std::endl;
            
            t += dt;
            y << attitude.headingMagneto;
            u = cnt.wx;
            kalmanFilter.update(y,u);
            //std::cout << "t = " << t << ", " << "y[" << i << "] = " << y.transpose()
                //<< ", x_hat[" << i << "] = " << kf.state().transpose() << std::endl;
            position_data pose;
            pose.yaw = kalmanFilter.state()[0];
            topic_position_data.publish(pose);
        }
    }

KalmanThread kalmanThread;