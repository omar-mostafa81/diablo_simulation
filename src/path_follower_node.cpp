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

using namespace std;

class DiabloSquarePathNode : public rclcpp::Node
{
public:
    DiabloSquarePathNode() : Node("path_follower_node")
    {
        RCLCPP_INFO(this->get_logger(), "Node Started!");

        // Read and update file index
        int file_index = readAndUpdateFileIndex();

        // Generate unique filename
        std::stringstream file_name_ss;
        file_name_ss << "/home/nyukl/Desktop/matlab/sim/sim_test_" << file_index << ".txt";
        file_path_ = file_name_ss.str();

        diablo_pose_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/simulation/odom", 1000, std::bind(&DiabloSquarePathNode::poseCallback, this, std::placeholders::_1));

        motion_publisher_ = this->create_publisher<motion_msgs::msg::MotionCtrl>(
            "/simulation/velocity", 1000);

        velocity_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/non_filtered/velocity", 1000);

        // Initialize control parameters
        Kv = 0.4;       // Forward gain factor
        Kw = 1.0;      // Rotation gain factor
        
        initial_x = 0.0;
        initial_y = 0.0;
        x_c = 0.0;
        y_c = 0.0;
        yaw_c = 0.0;
        V_x = 0;
        V_y = 0;
        last_omega = 0;
        num_points = 5000;
        last_v = 0;
        linear_v = 0;
        first_loop = true;
        initialized = false;

        curv = 0;

        Dv_y = 0; 
        Dv_x = 0;
        shortest_d = 0;

        firstMsg = true;

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&DiabloSquarePathNode::start, this));

        // Create a new file for each run
        std::ofstream outfile(file_path_);
        RCLCPP_INFO(this->get_logger(), "File Created: %s", file_path_.c_str());
        outfile << "path_vector_x, path_vector_y" << std::endl;
        outfile.close();
    }

private:

    int readAndUpdateFileIndex()
    {
        std::string index_file_path = "/home/nyukl/Desktop/matlab/sim/file_index.txt";
        int file_index = 0;

        // Read the current index from the file
        std::ifstream infile(index_file_path);
        if (infile.is_open() && infile >> file_index)
        {
            infile.close();
        }

        // Increment and update the index for next run
        std::ofstream outfile(index_file_path);
        outfile << file_index + 1;
        outfile.close();

        return file_index;
    }

    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract Theta
        double yaw = msg->pose.pose.orientation.x;

        //Extract position readings
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        //Extract Velocity readings
        V_x = msg->twist.twist.linear.x;
        V_y = msg->twist.twist.linear.y;

        x_c = x;
        y_c = y;
        yaw_c = yaw;
       

        if (firstMsg)
        {
            initial_x = x;
            initial_y = y;
            firstMsg = false;
        }

    }

    void start() {
        
        if (!initialized) {
            //Create the path
            path_vector = create_circle_vector(0.1, 0.1, 0.6, num_points);
            initialized = true;
            RCLCPP_INFO(this->get_logger(), "Path is created");
        }

        // pair<double, double> desired_velocity = calculateDesiredVelocity(path_vector, x_c, y_c);
        // Dv_x = desired_velocity.first;
        // Dv_y = desired_velocity.second;
        
        move_to (x_c, y_c, V_x, V_y);

    }

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
            double x = x_center + radius * 1 * std::cos(2 * M_PI * i / num_points) + 1* std::cos(2* 2 * M_PI * i / num_points);
            double y = y_center + radius * 1 * std::sin(2 * M_PI * i / num_points) + 0.1* std::sin(2* 2 * M_PI * i / num_points);
            // + 0.8* std::cos(2* 2 * M_PI * i / num_points)
            // + 0.05* std::sin(2* 2 * M_PI * i / num_points)

            circle_vector.emplace_back(x, y); 

        }
        
        return circle_vector;
    }

    pair<double, double> calculateDesiredVelocity(const vector<pair<double, double>>& path_vector, double x_c, double y_c) {
        //Calculate the Shortest distance to the path
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
        double N_x2= 0; 
        double N_y2 = 0;
        double T_x=0;
        double T_y=0;
        double T2_x=0;
        double T2_y=0;
        double dT_x = 0;
        double dT_y = 0;
        double ds = 0;
        double aux_rho;
        curv = 0;
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
    
            //find the next tangent vector for curvature calculation
            double Tx_a2 = (path_vector[(i+2)%path_vector.size()].first - path_vector[(i+1)%path_vector.size()].first);
            double Ty_a2 = (path_vector[(i+2)%path_vector.size()].second - path_vector[(i+1)%path_vector.size()].second);
            double T2_norm = sqrt(Tx_a2*Tx_a2 + Ty_a2*Ty_a2);

            T2_x += aux_rho*Tx_a2/(T2_norm+1e-5);
            T2_y += aux_rho*Ty_a2/(T2_norm+1e-5);

            curv += aux_rho*( Tx_a *(Ty_a2-Ty_a) - Ty_a*(Tx_a2-Tx_a))/pow(T_norm,3);

            ds = (T_norm + T2_norm)/2;
            
            rho_sum +=  aux_rho;
        }

        curv = curv/rho_sum;

        N_x = N_x/rho_sum;
        N_y = N_y/rho_sum;
        T_x = T_x/rho_sum;
        T_y = T_y/rho_sum;
        T2_x = T2_x/rho_sum;
        T2_y = T2_y/rho_sum;

        dT_x = (T2_x - T_x)/ds;
        dT_y = (T2_y - T_y)/ds;

        //calculate another normal that has fixed direction of the vectors (either always inward or outward)
        N_x2 = (-1) * T_y;
        N_y2 = T_x;

        //Calculate the curvature
        //curv =  (N_x2 * dT_x) + (N_y2 * dT_y);
        
        //cout << "proj:"<<N_x*T_x+N_y*T_y<<std::endl;

        RCLCPP_INFO(this->get_logger(), "Distance to the path is: %.2f \n",shortest_d);

        //calculate the Normal
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

// not needed part:
        // //Calculate the curvature
        // double ds = (T + T_next)/2;
        // double curv = (T_next - T)/ds;
        // RCLCPP_INFO(this->get_logger(), "Curvature is: %.2f \n",curv);

        //trail
        // double T_x = 0, T_y = 0;
        // double T_next_x = 0, T_next_y = 0;

        // if (mag > 0) {
        //     T_x = (path_vector[next_index].first - path_vector[index_s].first) / mag;
        //     T_y = (path_vector[next_index].second - path_vector[index_s].second) / mag;
        // }

        // if (mag_next > 0) {
        //     T_next_x = (path_vector[next_next_index].first - path_vector[next_index].first) / mag_next;
        //     T_next_y = (path_vector[next_next_index].second - path_vector[next_index].second) / mag_next;
        // }

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

//        RCLCPP_INFO(this->get_logger(), "Curvature is: %.2f \n",curv);


        // Calculate the linear combination weights
        //double k_0 = 0.01;
        //double alpha = (2/M_PI) * atan(30*pow(shortest_d,2));
        double alpha = (2/M_PI) * atan(60*pow(shortest_d,2));
        double beta = sqrt(1-pow(alpha,2));
        // double alpha = shortest_d / sqrt(pow(shortest_d, 2) + pow(k_0, 2));
        // double beta = k_0 / sqrt(pow(shortest_d, 2) + pow(k_0, 2));

        // Calculate the desired velocity components
        double v_max = 0.3;
        double curv_max = 1;
        double mult = 0;

        if (curv < curv_max) {
            mult = v_max;
        } else if (curv > 9) {
            mult = v_max * 0.2;
        } else {
            mult = (2*curv_max*v_max)/(curv+curv_max);
        }

        //RCLCPP_INFO(this->get_logger(), "Mult is: %.2f \n",mult);
        //mult depends on the curvature, represents the linear v
        double Dv_x = mult * ((alpha * N_x) + (beta * T_x));
        double Dv_y = mult * ((alpha * N_y) + (beta * T_y));

        //double Dv_x = ((alpha * N_x) + (beta * T_x));
        //double Dv_y = ((alpha * N_y) + (beta * T_y));


        //REMOVE
        // double dx=x_c-0.5;
        // double dy=y_c;

        // double DD = dx*dx+dy*dy-0.6*0.6;
        // Dv_x = v_max*(-(dx*DD) + dy);
        // Dv_y = v_max*(-(dy*DD) - dx);

        //RCLCPP_INFO(this->get_logger(), "Dv_x is: %.2f \n",Dv_x);
        //RCLCPP_INFO(this->get_logger(), "Dv_y is: %.2f \n",Dv_y);
        
        return {Dv_x, Dv_y};
    }

    void move_to (double xc, double yc, double V_x, double V_y) {

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
        double J[2][2];

        J[0][0] = (Vdx_xp - Vdx_xn)/(2*u);
        J[1][0] = (Vdx_yp - Vdx_yn)/(2*u);
        J[0][1] = (Vdy_xp - Vdy_xn)/(2*u);
        J[1][1] = (Vdy_yp - Vdy_yn)/(2*u);
        

        //RCLCPP_INFO(this->get_logger(), "J00 : %.2f \n",J[0][0]);
        //RCLCPP_INFO(this->get_logger(), "J10 : %.2f \n",J[1][0]);
        //RCLCPP_INFO(this->get_logger(), "J01 : %.2f \n",J[0][1]);
        //RCLCPP_INFO(this->get_logger(), "J11 : %.2f \n",J[1][1]);

        //Save the current velocity vector into a Matrix for matrix multiplication
        double v[2][1];
        v[0][0] = V_x;
        v[1][0] = V_y;
        //RCLCPP_INFO(this->get_logger(), "Current V_x : %.2f \n",V_x);
        //RCLCPP_INFO(this->get_logger(), "Current V_y : %.2f \n",V_y);
        //Perform Matrix multiplication to Obtain Ad_x, Ad_y
        double Ad_x = (J[0][0] * v[0][0]) + (J[0][1] * v[1][0]);
        double Ad_y = (J[1][0] * v[0][0]) + (J[1][1] * v[1][0]);

        //Perfom error correction to obtain Ax, Ay
        double c_factor = 0.2;
        double Ax = Ad_x - (c_factor * (V_x - Vd_x));
        double Ay = Ad_y - (c_factor * (V_y - Vd_y));

        double normA  = sqrt(Ax*Ax+Ay*Ay);
        if (normA >= 0.3)
        {
            Ax = (0.3*Ax)/normA;
            Ay = (0.3*Ay)/normA;
        }       
        
        //Find V_dot and omega
        double V_dot = (cos(yaw_c) * Ax) + (sin(yaw_c)* Ay);
        //Calculare linear velocity from v_dot
        linear_v = last_v + (V_dot * 0.1);

        //at the first few seconds, the angular_v is too large because the last_v is too small

        //change last_v in the robot to have abs
        double angular_v = 0;
        if (abs(last_v) >= 0.05) {
            angular_v = (((-1) * sin(yaw_c) * Ax) + (cos(yaw_c) * Ay)) / (last_v); 
        } else {
            angular_v = (((-1) * sin(yaw_c) * Ax) + (cos(yaw_c) * Ay)) / (0.05*(last_v>=0?1:-1));
        }

        double fact = 1;
        double max_w = 1.0;

        if(abs(angular_v) > max_w) {
            fact = max_w/abs(angular_v);
        } 
        angular_v *= fact;
        linear_v *= fact; 

        // double alpha = 0.0;
        // linear_v = alpha * last_v + (1-alpha) * linear_v;
        // angular_v = alpha * last_omega + (1-alpha) * angular_v;

        std::ofstream outfile;
        outfile.open(file_path_, std::ios_base::app);
        
        // Write data in columns separated by commas
        rclcpp::Time now = rclcpp::Clock().now();
        int64_t milliseconds = now.nanoseconds() / 1000000;

	    //linear_v = 0.3*sin(0.3*milliseconds/(1000));
	    //angular_v = 0.3*sin(0.1*milliseconds/(1000));

        if (first_loop) {
            //record the path vector
            for (size_t i = 0; i < path_vector.size(); ++i) {
                outfile << path_vector[i].first << ",";
                outfile << path_vector[i].second << endl;
            }

            //add the titles to the data  
            outfile << "Timestamp_millisec,Position_X,Position_Y,Orientation_Z, current_Vel_x, current_Vel_y, desired_Velocity_x, desired_Velocity_y, Target linear_velocity, Target Angular_v, Target Accel, curv" << std::endl;

            first_loop = false;
        }

        outfile << milliseconds << ",";
        outfile << x_c << ",";
        outfile << y_c << ",";
        outfile << yaw_c << ",";
	    outfile << V_x << ",";
	    outfile << V_y << ",";
        outfile << Vd_x << ",";
        outfile << Vd_y << ",";
        outfile << linear_v << ",";
        outfile << angular_v << ",";
         outfile << V_dot << ",";
        outfile << curv << std::endl;

        outfile.close();
        

        RCLCPP_INFO(this->get_logger(), "Prompted Linear V: %.2f \n",linear_v);
        RCLCPP_INFO(this->get_logger(), "Promped Angular V: %.2f \n", angular_v);

        motion_msgs::msg::MotionCtrl cmd;
        cmd.mode_mark = false;
        cmd.value.forward = linear_v;
        cmd.value.left = angular_v;
        cmd.value.up = 1;
        motion_publisher_->publish(cmd);

        last_v = linear_v;
        last_omega = angular_v;

    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr diablo_pose_subscriber_;
    rclcpp::Publisher<motion_msgs::msg::MotionCtrl>::SharedPtr motion_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool  firstMsg;
    double initial_x;
    double initial_y;
    double Kv;
    double Kw;
    double last_omega;
    double x_c, y_c, yaw_c, V_x, V_y;
    double last_v;
    double Dv_y, Dv_x;
    double target_x, target_y;
    double shortest_d;
    vector<pair<double, double>> path_vector;
    int num_points;
    bool initialized;
    double linear_v;
    std::string file_path_;
    bool first_loop;
    double curv;


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiabloSquarePathNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

