#pragma once

#include "main.hpp"

#include <vector>
#include <Eigen/Dense>
#include "kalman.hpp"

struct KalmanThread : public Subscriber, public StaticThread<> {

    CommBuffer<imu_data> inputMsgBuffer;

    KalmanFilter kalmanFilter;
    int n = 2; // Number of states
    int m = 1; // Number of measurements
    int nu = 1; //Number of system inputs
    double R_Gyro = 0.05;
    float u = 1;
    double t = 0;
    double dt = 1.0/50; // Time step

    KalmanThread();

    uint32_t put(const uint32_t topicId, const size_t len, void *data, [[gnu::unused]] const NetMsgInfo &netMsgInfo);

    void init();

    void run();
};