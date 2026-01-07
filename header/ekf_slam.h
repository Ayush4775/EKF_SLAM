#include <Eigen/Dense>
#include <unordered_map>
#include <fstream>

class EKFSLAM {
    public:
        EKFSLAM();
        ~EKFSLAM();

        void initialize();
        void predict(double v, double w, double dt);
        void update(int landmark_id, double range, double bearing);
    private:
        Eigen::VectorXd mean;
        Eigen::MatrixXd Sigma;
        std::unordered_map<int, std::size_t> landmark_indices;
        std::ofstream state_file;
};