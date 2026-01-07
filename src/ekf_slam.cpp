#include "ekf_slam.h"
#include <iostream>
#include <unordered_map>

EKFSLAM::EKFSLAM() {
    // Initial robot pose only: x, y, theta
    mean = Eigen::VectorXd::Zero(3);
    Sigma = Eigen::MatrixXd::Identity(3, 3) * 0.01;
    state_file.open("ekf_slam_state.csv");
}
EKFSLAM::~EKFSLAM() {
    // Destructor
}

void EKFSLAM::predict(double v, double w, double dt) {
    float theta = mean(2);
    Eigen::VectorXd u(3);
    Eigen::MatrixXd Jacobian = Eigen::MatrixXd::Identity(mean.size(), mean.size());
    if (std::abs(w) > 1e-5) {
        u(0) = (v / w) * (std::sin(theta + w * dt) - std::sin(theta));
        u(1) = (v / w) * (-std::cos(theta + w * dt) + std::cos(theta));
        Jacobian(0, 2) = (v / w) * (std::cos(theta + w * dt) - std::cos(theta));
        Jacobian(1, 2) = (v / w) * (std::sin(theta + w * dt) - std::sin(theta));
    } else {
        u(0) = v * dt * std::cos(theta);
        u(1) = v * dt * std::sin(theta);
        Jacobian(0, 2) = -v * dt * std::sin(theta);
        Jacobian(1, 2) = v * dt * std::cos(theta);
    }
    u(2) = w * dt;
    // normalize u((2)
    u(2) = std::atan2(std::sin(u(2)), std::cos(u(2)));
    
    mean.head(3) += u;
    mean(2) = std::atan2(std::sin(mean(2)), std::cos(mean(2)));
    Sigma = Jacobian * Sigma * Jacobian.transpose();
    Eigen::MatrixXd motion_noise = Eigen::MatrixXd::Zero(mean.size(), mean.size());
    motion_noise(0, 0) = abs(v) * abs(w) * dt * 0.1;
    motion_noise(1, 1) = abs(v) * abs(w) * dt * 0.1;
    motion_noise(2, 2) = abs(w) * dt * 0.01;

    Sigma += motion_noise;
    Sigma = 0.5 * (Sigma + Sigma.transpose());

    
    for(int i =0; i<mean.size(); ++i) {
        state_file << mean(i);
        if(i != mean.size() -1) state_file << ",";
    }
    state_file << "\n";
    
}

void EKFSLAM::update(int landmark_id, double range, double bearing) {
    int landmark_index;
    if (landmark_indices.find(landmark_id) == landmark_indices.end()) {
        landmark_index = mean.size();
        landmark_indices[landmark_id] = landmark_index;
        mean.conservativeResize(landmark_index + 2);
        Sigma.conservativeResize(landmark_index + 2, landmark_index + 2);
        mean(landmark_index) = mean(0) + range * cos(bearing + mean(2));
        mean(landmark_index + 1) = mean(1) + range * sin(bearing + mean(2));
        Sigma.block(landmark_index, 0, 2, Sigma.cols()).setZero();
        Sigma.block(0, landmark_index, Sigma.rows(), 2).setZero();
        Sigma(landmark_index, landmark_index) = 1.0;
        Sigma(landmark_index + 1, landmark_index + 1) = 1.0;
        Sigma(landmark_index, landmark_index + 1) = 0.0;
        Sigma(landmark_index + 1, landmark_index) = 0.0;
    
    }
    else {
        landmark_index = landmark_indices[landmark_id];
        auto dx = mean(landmark_index) - mean(0);
        auto dy = mean(landmark_index + 1) - mean(1);
        auto q = dx * dx + dy * dy;
        auto sqrt_q = std::sqrt(q);
        auto predicted_bearing = std::atan2(dy, dx) - mean(2);
        predicted_bearing = std::atan2(std::sin(predicted_bearing), std::cos(predicted_bearing));
        Eigen::Vector2d z_pred;
        z_pred(0) = sqrt_q;
        z_pred(1) = predicted_bearing;
        Eigen::Vector2d z;
        z(0) = range;
        z(1) = bearing;
        Eigen::Vector2d y = z - z_pred;
        y(1) = std::atan2(std::sin(y(1)), std::cos(y(1)));
        Eigen::MatrixXd H(2, mean.size());
        H.setZero();
        H(0, 0) = -dx / sqrt_q ;
        H(0, 1) = -dy / sqrt_q ;
        H(0, landmark_index) = dx / sqrt_q ;
        H(0, landmark_index + 1) = dy / sqrt_q ;
        H(1, 0) = dy / q ;
        H(1,1) = -dx / q ;
        H(1,2) = -1;
        H(1, landmark_index) = -dy / q ;
        H(1, landmark_index + 1) = dx / q ;
        Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
        R(0, 0) = 0.5;
        R(1, 1) = 0.05;
        Eigen::Matrix2d S = H * Sigma * H.transpose() + R;
        Eigen::MatrixXd K = Sigma * H.transpose() * S.inverse();
        mean += K * y;
        Sigma = (Eigen::MatrixXd::Identity(mean.size(), mean.size()) - K * H) * Sigma;
        Sigma = 0.5 * (Sigma + Sigma.transpose());

        mean(2) = std::atan2(std::sin(mean(2)), std::cos(mean(2)));
        if (K(0,0) > 0.4 || K(1,0) > 0.4) {
            std::cout << "Large Kalman Gain detected!" << std::endl;
        }



    }

    // std::cout << "Predicted Mean: " << mean.transpose() << std::endl;
    // std::cout << "Predicted mean head: " <<mean.head(3).transpose() << std::endl;


}
