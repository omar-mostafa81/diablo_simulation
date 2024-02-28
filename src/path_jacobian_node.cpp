#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "motion_msgs/msg/motion_ctrl.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>
#include <utility> 
#include <fstream>
#include <filesystem>
#include <sstream>
#include <nav_msgs/msg/odometry.hpp>
#include <fstream>
#include <vector>
#include <cmath>
#include <filesystem>

using namespace std;

class PathJacobianNode : public rclcpp::Node {
public:
    PathJacobianNode() : Node("path_jacobian_node") { 
        calculate_and_save_jacobian();
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    vector<pair<double, double>> path_vector = create_circle_vector(0.1, 0.1, 0.6, 1000);

    std::vector<std::pair<double, double>> create_circle_vector(double x_center, double y_center, double radius, int num_points) {
        std::vector<std::pair<double, double>> circle_vector;
        circle_vector.reserve(num_points); // Reserve space for num_points elements
        
        for (int i = 0; i < num_points; ++i) {
	   // Calculate the angle for this point
            double angle = 2 * M_PI * i / num_points;
            
            // Adjust the radius to create petal effects
            // The '4' controls the number of petals
            // Adjusting the amplitude (e.g., radius * 0.2) controls the "softness" of the petals
            double adjusted_radius = radius + radius * 0.2 * std::cos(4 * angle);
            
            // Calculate the position using the adjusted radius
            //double x = x_center + adjusted_radius * std::cos(angle);
            //double y = y_center + adjusted_radius * std::sin(angle);
            double x = x_center + radius * 1 * std::cos(2 * M_PI * i / num_points) + 0.8* std::cos(2* 2 * M_PI * i / num_points);
            double y = y_center + radius * 1 * std::sin(2 * M_PI * i / num_points)  + 0.05* std::sin(2* 2 * M_PI * i / num_points);
            // + 0.8* std::cos(2* 2 * M_PI * i / num_points)
            // + 0.05* std::sin(2* 2 * M_PI * i / num_points)

            circle_vector.emplace_back(x, y); 

        }
        
        return circle_vector;
    }

    void calculate_and_save_jacobian() {
    double x_min = -2, x_max = 2, y_min = -2, y_max = 2;
    int N = 400;
    
    // Specify your absolute path here
    std::string specified_absolute_path = "/home/nyukl/bag/jacobian_data/";
    std::filesystem::path dir_path(specified_absolute_path);

    // Create directory if it does not exist
    std::filesystem::create_directories(dir_path);

    std::ofstream fileJ00(dir_path / "J00.txt"), fileJ01(dir_path / "J01.txt"), fileJ10(dir_path / "J10.txt"), fileJ11(dir_path / "J11.txt");

    // Check if files are successfully opened
    if (!fileJ00.is_open() || !fileJ01.is_open() || !fileJ10.is_open() || !fileJ11.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open one or more files.");
        return; // Exit the function if any file failed to open
    }
    RCLCPP_INFO(this->get_logger(), "Opened the files");
    for (int i = 0; i < N; ++i) {
        double X = x_min + (x_max - x_min) * i / (N - 1);
        for (int j = 0; j < N; ++j) {
            double Y = y_min + (y_max - y_min) * j / (N - 1);

            vector<vector<double>> J = calculateJacobianAtPoint(X, Y);
            fileJ00 << J[0][0] << ";";
            fileJ01 << J[0][1] << ";";
            fileJ10 << J[1][0] << ";";
            fileJ11 << J[1][1] << ";";

            RCLCPP_INFO(this->get_logger(), "J00 is: %.2f \n",J[0][0]);
    
        }
        fileJ00 << std::endl;
        fileJ01 << std::endl;
        fileJ10 << std::endl;
        fileJ11 << std::endl;
    }

    RCLCPP_INFO(this->get_logger(), "Finished writing to files.");
    fileJ00.close();
    fileJ01.close();
    fileJ10.close();
    fileJ11.close();
}


    // Placeholder for the method to calculate the Jacobian
    // You would need to implement this based on the provided logic
    vector<vector<double>> calculateJacobianAtPoint(double xc, double yc) {
        //Get VF(x,y) and VF(x+u, y) and VF(x, y+u)
        double u = 0.01;
        pair<double, double> Vd = calculateDesiredVelocity(path_vector, xc, yc);
        double Vd_x = Vd.first;
        double Vd_y = Vd.second;

        pair<double, double> Vdx = calculateDesiredVelocity(path_vector, xc+u, yc);
        double Vdx_xp = Vdx.first;
        double Vdx_yp = Vdx.second;

        pair<double, double> Vdy = calculateDesiredVelocity(path_vector, xc, yc+u);
        double Vdy_xp = Vdy.first;
        double Vdy_yp = Vdy.second;

        pair<double, double> Vdx_n = calculateDesiredVelocity(path_vector, xc-u, yc);
        double Vdx_xn = Vdx_n.first;
        double Vdx_yn = Vdx_n.second;

        pair<double, double> Vdy_n = calculateDesiredVelocity(path_vector, xc, yc-u);
        double Vdy_xn = Vdy_n.first;
        double Vdy_yn = Vdy_n.second;

        
        //Calculate Jacobian Matrix
        vector<vector<double>> J(2, vector<double>(2));
        J[0][0] = (Vdx_xp - Vdx_xn) / (2 * u);
        J[1][0] = (Vdx_yp - Vdx_yn) / (2 * u);
        J[0][1] = (Vdy_xp - Vdy_xn) / (2 * u);
        J[1][1] = (Vdy_yp - Vdy_yn) / (2 * u);

        return J;
    }

    double shortest_d = 0;
    pair<double, double> calculateDesiredVelocity(const vector<pair<double, double>>& path_vector, double x_c, double y_c) {

        shortest_d = numeric_limits<double>::max();
        double distance = 0;
        size_t index_s = 0;
        
        vector<double> all_distance = {};
        double h = 0.01;


        for (size_t i = 0; i < path_vector.size(); ++i) {
            distance = sqrt(pow(path_vector[i].first - x_c, 2) + pow(path_vector[i].second - y_c, 2));
            if (distance < shortest_d) {
                shortest_d = distance;
                index_s = i;
            }
            all_distance.push_back(distance);
        }

        vector<double> all_rho = {};
        double rho_sum = 0;
        double N_x=0;
        double N_y=0;
        double T_x=0;
        double T_y=0;
        double aux_rho;
        for(int i=0; i < path_vector.size(); i++)
        {
            aux_rho = exp(-(all_distance[i]-shortest_d)/h);
            N_x += aux_rho*(path_vector[i].first - x_c)/(all_distance[i]+1e-5);
            N_y += aux_rho*(path_vector[i].second - y_c)/(all_distance[i]+1e-5);

            double Tx_a = (path_vector[(i+1)%path_vector.size()].first - path_vector[i].first);
            double Ty_a = (path_vector[(i+1)%path_vector.size()].second - path_vector[i].second);
            double T_norm = sqrt(Tx_a*Tx_a + Ty_a*Ty_a);

            T_x += aux_rho*Tx_a/(T_norm+1e-5);
            T_y += aux_rho*Ty_a/(T_norm+1e-5);
            rho_sum +=  aux_rho;

        }

        

        N_x = N_x/rho_sum;
        N_y = N_y/rho_sum;
        T_x = T_x/rho_sum;
        T_y = T_y/rho_sum;

        cout << "proj:"<<N_x*T_x+N_y*T_y<<std::endl;



        RCLCPP_INFO(this->get_logger(), "Distance to the path is: %.2f \n",shortest_d);
        //Calculate the Shortest distance to the path
        // double shortest_d = numeric_limits<double>::max();
        // double distance = 0;
        // size_t index_s = 0;
        
        // for (size_t i = 0; i < path_vector.size(); ++i) {
        //     distance = sqrt(pow(path_vector[i].first - x_c, 2) + pow(path_vector[i].second - y_c, 2));
        //     if (distance < shortest_d) {
        //         shortest_d = distance;
        //         index_s = i;
        //     }
        // }

        // //calculate the Normal
        // double N_x = 0;
        // double N_y = 0;
        
        // if (shortest_d != 0) {
        //     N_x = (path_vector[index_s].first - x_c) / (shortest_d);
        //     N_y = (path_vector[index_s].second - y_c) / (shortest_d);
        // }

        // size_t next_index = (index_s + 1) % path_vector.size();
        // // Calculate the magnitude of the vector difference
        // double mag = sqrt(pow(path_vector[next_index].first - path_vector[index_s].first, 2) +
        //                     pow(path_vector[next_index].second - path_vector[index_s].second, 2));

        // // Calculate the unit vector components in the tangent direction
        // double T_x = (path_vector[next_index].first - path_vector[index_s].first) / (mag+0.000001);
        // double T_y = (path_vector[next_index].second - path_vector[index_s].second) / (mag+0.00001);

        // double T = sqrt(pow(T_x, 2) + pow(T_y, 2));

        // size_t next_next_index = (next_index + 1) % path_vector.size();
        // // Calculate the magnitude of the vector difference
        // double mag_next = sqrt(pow(path_vector[next_next_index].first - path_vector[next_index].first, 2) +
        //                     pow(path_vector[next_next_index].second - path_vector[next_index].second, 2));

        // // Calculate the unit vector components in the tangent direction
        // double T_next_x = (path_vector[next_next_index].first - path_vector[next_index].first) / (mag_next+0.000001);
        // double T_next_y = (path_vector[next_next_index].second - path_vector[next_index].second) / (mag_next+0.00001);

        // double T_next = sqrt(pow(T_next_x, 2) + pow(T_next_y, 2));

        // // Calculate the change in the tangent vector components
        // double delta_T_x = T_next_x - T_x;
        // double delta_T_y = T_next_y - T_y;

        // // Calculate the norm of the change in tangent vector
        // double norm_delta_T = sqrt(pow(delta_T_x, 2) + pow(delta_T_y, 2));

        // //Calculate the curvature
        // double ds = (mag + mag_next) / 2; // Average segment length
        // double curv = 0;
        // if (ds > 0) {
        //     curv = norm_delta_T / ds;
        // }

        // Calculate the linear combination weights
        //double k_0 = 0.01;
        double alpha = (2/M_PI) * atan(30*pow(shortest_d,2));
        double beta = sqrt(1-pow(alpha,2));
        // double alpha = shortest_d / sqrt(pow(shortest_d, 2) + pow(k_0, 2));
        // double beta = k_0 / sqrt(pow(shortest_d, 2) + pow(k_0, 2));

        // Calculate the desired velocity components
        //double mult;
        double v_max = 0.3;
        double curv_max = 1;
        double mult = 0;

        // if (curv < curv_max) {
        //     mult = v_max;
        // } else if (curv > 9) {
        //     mult = v_max * 0.2;
        // } else {
        //     mult = (2*curv_max*v_max)/(curv+curv_max);
        // }
        //mult depends on the curvature, represents the linear v
        double Dv_x = v_max * ((alpha * N_x) + (beta * T_x));
        double Dv_y = v_max * ((alpha * N_y) + (beta * T_y));

        //double Dv_x = ((alpha * N_x) + (beta * T_x));
        //double Dv_y = ((alpha * N_y) + (beta * T_y));

        
        return {Dv_x, Dv_y};
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<PathJacobianNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
