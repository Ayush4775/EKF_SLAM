#include <iostream>
#include "config.h"
#include "ekf_slam.h"
#include "sim.h"
#include <random>
#include <vector>
#include <cstdlib>
#include <fstream>


Pos2D step_truth(Pos2D p, Control u) {
    // Simple unicycle model
    Pos2D p_new;
    if (std::abs(u.w) > 1e-5) { 
        p_new.x = p.x + (u.v / u.w) * (std::sin(p.theta + u.w * u.dt) - std::sin(p.theta));
        p_new.y = p.y + (u.v / u.w) * (-std::cos(p.theta + u.w * u.dt) + std::cos(p.theta));
    } else {
        p_new.x = p.x + u.v * u.dt * std::cos(p.theta);
        p_new.y = p.y + u.v * u.dt * std::sin(p.theta);
    }
    p_new.theta = p.theta + u.w * u.dt;
    p_new.theta = std::atan2(std::sin(p_new.theta), std::cos(p_new.theta));
    return p_new;
}
Control add_noise(Control u) {
    Control u_noisy;
    std::mt19937 rng(42);
    std::normal_distribution<float> d1(0.0f, 0.1f);
    std::normal_distribution<float> d2(0.0f, 0.02f);
    u_noisy.v = u.v + d1(rng);
    u_noisy.w = u.w + d2(rng);
    u_noisy.dt = u.dt;
    return u_noisy;
}

std::vector<Observation> get_observations(Pos2D p, const std::vector<Landmark>& landmarks) {
    // Use global variables for max_range and fov

    extern const float max_range;
    extern const float fov;

    std::vector<Observation> observations;
    std::mt19937 rng(42);
    std::normal_distribution<float> range_dist(0.0f, 2.0f);
    std::normal_distribution<float> bearing_dist(0.0f, 0.1f);
    for (const auto& lm : landmarks) {

        float dx = lm.x - p.x;
        float dy = lm.y - p.y;
        float range = std::sqrt(dx * dx + dy * dy);
        float bearing = std::atan2(dy, dx) - p.theta;
        if (range <= max_range && bearing >= -fov / 2 && bearing <= fov / 2) {
            
            Observation obs;
            obs.landmark_id = lm.id;
            obs.range = range + range_dist(rng); // add small noise
            obs.bearing = bearing + bearing_dist(rng); // add small noise
            observations.push_back(obs);
        }
    }
    return observations;
}

void generate_landmarks(std::vector<Landmark>& landmarks, int num_landmarks = 100) {
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(-15.0f, 15.0f);
    std::fstream file("landmarks.csv", std::ios::out | std::ios::trunc);
    file << "id,x,y\n";
    for (int i = 0; i < num_landmarks; ++i) {
        Landmark lm;
        lm.id = i;
        lm.x = dist(rng);
        lm.y = dist(rng);
        landmarks.push_back(lm);
        file << lm.id << "," << lm.x << "," << lm.y << "\n";
    }
}
void generate_controls(std::vector<Control>& controls, int num_controls = 500) {
    for(int i = 0; i < num_controls; ++i) {
        Control u;
        u.v = 5.0f; // constant forward velocity
        u.w = 0.1f * ((i % 30) - 10); // oscillating angular velocity
        u.dt = 0.1f; // time step
        controls.push_back(u);
    }
}
int main() {
    std::vector<Landmark> landmarks;
    std::ofstream file("robot.csv");

    std::srand(42); // For reproducibility
    generate_landmarks(landmarks);
    EKFSLAM ekf_slam;
    std::vector<Control> controls;
    generate_controls(controls);
    Pos2D p_true = {0.0f, 0.0f, 0.0f};
    int i =0;
    file << "time,x,y,theta\n";
    for(const auto& u : controls) {
        file << i << "," << p_true.x << "," << p_true.y << "," << p_true.theta << std::endl;
        p_true = step_truth(p_true, u);
        Control u_noisy = add_noise(u);
        std::vector<Observation> observations = get_observations(p_true, landmarks);
        ekf_slam.predict(u_noisy.v, u_noisy.w, u_noisy.dt);
        for (const auto& obs : observations) {
            ekf_slam.update(obs.landmark_id, obs.range, obs.bearing);
        }

        ++i;
    }




    std::cout << "EKF-SLAM project setup complete." << std::endl;
    return 0;
}
