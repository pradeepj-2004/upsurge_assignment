#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "quadrupled_control/srv/ik_service.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using namespace std;
using Vector3 = std::array<double, 3>;
using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;

using namespace std::chrono_literals;
using IKService = quadrupled_control::srv::IKService;

class RobotControllerNode : public rclcpp::Node
{

struct traj_points {
double x;
double y;
double z;};

struct waypoints {
double theta1;
double theta2;
double theta3;
double theta4;
double theta5;
double theta6;
double theta7;
double theta8;
double theta9;
double theta10;
double theta11;
double theta12;};


public:
  RobotControllerNode() : Node("robot_controller_node")
  {

    //Initalising Subscriber from cmd_vel topic
    callback_group_subscriber = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_subscriber;


    cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1,std::bind(&RobotControllerNode::cmd_vel_callback, this, std::placeholders::_1),sub_opt);


    //Initalising Client for IK
    callback_group_client = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_ = this->create_client<IKService>("solve_ik", rmw_qos_profile_services_default, callback_group_client);

    //Initalising Joint angle Publisher
    joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_group_effort_controller/joint_trajectory", 10);
    

    //Checking for service
    while (!client_->wait_for_service(10s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for the solve_ik service to be available...");
    }
    
    // Initial position setting
    rclcpp::Rate slow_loop_(1);
    slow_loop_.sleep();
    send_trajectory_point(0.0, 1.138140, -1.838006,0.0, 1.138140, -1.838006,0.0, 1.138140, -1.838006,0.0, 1.138140, -1.838006,2);
    }

private:

    std::vector<double> linspace(double start, double end, size_t total_size)
    {
        std::vector<double> result(total_size);
        double step = (end - start) / (total_size - 1);

        for (size_t i = 0; i < total_size; ++i) {
            result[i] = start + i * step;}

        return result;
    }


    int nCr(int n, int k) {
        if (k == 0 || k == n) return 1;
        int res = 1;
        for (int i = 1; i <= k; i++) {
            res *= (n - (k - i));
            res /= i;
        }
        return res;
    }


    void compute_bezier_points(double d, double h, double theta, Eigen::Vector3d P3,int bezier_points,std::vector<traj_points> &trajectory_points,bool phase){

        double dx = d * 0.7 * cos(theta);
        double dy = d * 0.7 * sin(theta);


        // Define control points
        Eigen::Vector3d P0 = {P3(0), P3(1), P3(2) + h};
        Eigen::Vector3d P1 = {P3(0) + (4.0 / 5.0) * dx, P3(1) + (4.0 / 5.0) * dy, P3(2) + (3.0 / 5.0) * h};
        Eigen::Vector3d P2 = {P3(0) + (5.0 / 5.0) * dx, P3(1) + (5.0 / 5.0) * dy, P3(2) + (1.0 / 5.0) * h};
        Eigen::Vector3d P4 = {P3(0) - (5.0 / 5.0) * dx, P3(1) - (5.0 / 5.0) * dy, P3(2) + (1.0 / 5.0) * h};
        Eigen::Vector3d P5 = {P3(0) - (4.0 / 5.0) * dx, P3(1) - (4.0 / 5.0) * dy, P3(2) + (3.0 / 5.0) * h};
        Eigen::Vector3d P6 = P0;


        std::vector<double> u_values = {0.0, 0.125, 0.25, 0.5, 0.75, 0.875, 1.0};

        Eigen::MatrixXd V = Eigen::MatrixXd::Zero(7, 7);


        for (int j = 0; j < 7; j++) {  
            double u = u_values[j];
            for (int i = 0; i <= 6; i++) { 
                V(j, i) = nCr(6, i) * pow((1 - u), (6 - i)) * pow(u, i);
            }
        }


        Eigen::MatrixXd P(7, 3);
        P << P0.transpose(),P1.transpose(),P2.transpose(),P3.transpose(),P4.transpose(),P5.transpose(),P6.transpose();

        Eigen::MatrixXd W = (V.transpose() * V).colPivHouseholderQr().solve(V.transpose() * P);
        std::vector<double> u_vals = linspace(0.0, 1.0, bezier_points);

        if(phase==true){
        for (int i =0; i<bezier_points;i++){
            double u = u_vals[i];
            traj_points point = {0.0, 0.0, 0.0};
            for (int j = 0; j<7; j++){
                double bern_coeff = nCr(6, j) * pow((1-u), (6 - j)) * pow(u,j);
                traj_points W_row = { bern_coeff*W(j,0), bern_coeff*W(j,1), bern_coeff*W(j,2) };
                point.x = point.x + W_row.x;
                point.y = point.y + W_row.y;
                point.z = point.z + W_row.z;
            }
            trajectory_points.push_back(point);

        }
        }

        else{
            std::vector<traj_points> temp_points;
            for (int i =0; i<bezier_points;i++){
            double u = u_vals[i];
            traj_points point = {0.0, 0.0, 0.0};
            for (int j = 0; j<7; j++){
                double bern_coeff = nCr(6, j) * pow((1-u), (6 - j)) * pow(u,j);
                traj_points W_row = { bern_coeff*W(j,0), bern_coeff*W(j,1), bern_coeff*W(j,2) };
                point.x = point.x + W_row.x;
                point.y = point.y + W_row.y;
                point.z = point.z + W_row.z;
            }
            temp_points.push_back(point);
            }
            int point_index = static_cast<int>(temp_points.size()) / 2;

            for (int i = point_index; i < static_cast<int>(temp_points.size()); ++i) {
                trajectory_points.push_back(temp_points[i]);
            }
            for (int i = 0; i < point_index; ++i) {
                trajectory_points.push_back(temp_points[i]);
            }

        }

    }


    void send_trajectory_point(float theta1, float theta2, float theta3,float theta4,float theta5,float theta6,
       float theta7, float theta8, float theta9,float theta10,float theta11,float theta12,float time)
     {
      auto traj_msg = trajectory_msgs::msg::JointTrajectory();

       traj_msg.joint_names = {"lf_hip_joint","lf_upper_leg_joint","lf_lower_leg_joint",
       "rf_hip_joint","rf_upper_leg_joint","rf_lower_leg_joint","lh_hip_joint","lh_upper_leg_joint",
       "lh_lower_leg_joint","rh_hip_joint","rh_upper_leg_joint","rh_lower_leg_joint",};

      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = {theta1, theta2, theta3, theta4,theta5,theta6, theta7,theta8,theta9,theta10, theta11, theta12};  
   
      point.time_from_start = rclcpp::Duration::from_seconds(time);
      traj_msg.points.push_back(point);
      joint_trajectory_pub_->publish(traj_msg);

    }


    void leg_velocity(float linear_vel,float ang_vel){
        double v_x = linear_vel;
        double v_y = 0.0;
        double w = ang_vel;

        Eigen::Matrix<double, 3, 1> body_vel;
        body_vel << v_x,v_y,w;
        Eigen::Matrix<double, 4, 3> conversion_matrix;
        conversion_matrix << 1, 0, l/2,1, 0, -l/2,0, 1, 0,0, 1, -c;
        Eigen::Matrix<double, 4, 1> lateral_velocity = conversion_matrix * body_vel;

        double vd = lateral_velocity(0,0);
        double ve = lateral_velocity(1,0);
        double vf = lateral_velocity(2,0);
        double vt = lateral_velocity(3,0);
        double thrf = atan2(vf,(vd+0.0001));
        double thrb = atan2(vt,(vd+0.0001));
        double thlb = atan2(vt,(ve+0.0001));
        double thlf = atan2(vf,(ve+0.0001));

        leg_vel = {sqrt(vf*vf + vd*vd), sqrt(vt*vt + vd*vd), sqrt(vt*vt + ve*ve), sqrt(vf*vf + ve*ve), thrf, thrb, thlb, thlf}; 
    }


    std::vector<double> freq_cp(double vel, double cp_max1, double freq_min1, double freq_max1){
     
        std::vector<double> params;
        double cp_c = 0, freq_c = freq_min1;

        if (vel>freq_max1*cp_max1){
            params.push_back(cp_max1);
            params.push_back(freq_max1);
            return params;
        }
        else{
            for (; freq_c < freq_max1; freq_c = freq_c + 0.01){
                cp_c = vel/freq_c;
                if (cp_c<= cp_max1 && freq_c<=freq_max1){
                    break;
                }
            }
        }
        params.push_back(cp_c);
        params.push_back(freq_c);
        return params;
    }


    void cmd_vel_callback(const geometry_msgs::msg::Twist &msg)
    {
    float d_rf,d_rb,d_lb,d_lf;
    linear_vel=msg.linear.x;
    ang_vel=msg.angular.z;

    leg_velocity(linear_vel,ang_vel);

    double max_vel = 0;

    int index=0;
    for (int i=0;i<4;i++){
        if(max_vel<leg_vel[i]){
            max_vel = leg_vel[i];
            index = i;
            }
        }

    std::vector<double> leg_bezier = freq_cp(leg_vel[index], cp_max, freq_min, freq_max);


    d_rf = leg_vel[0]/leg_bezier[1];
    d_rb = leg_vel[1]/leg_bezier[1];
    d_lb = leg_vel[2]/leg_bezier[1];
    d_lf = leg_vel[3]/leg_bezier[1];

    h=0.03; 
    
    if (abs(linear_vel)>0.15 || abs(ang_vel)>0.15){
        d_rf =0.0;
        d_rb =0.0;
        d_lb =0.0;
        d_lf =0.0;
        h=0.0;
        RCLCPP_WARN(this->get_logger(), "Received cmd_vel is not in range of max_velocity");

    }

    trajectory_points_1.clear(); trajectory_points_2.clear(); trajectory_points_3.clear(); trajectory_points_4.clear();

    quadruple_low_level_control(d_lf,d_rf,d_lb,d_rb,h);

    }


    void quadruple_low_level_control(float stance_1,float stance_2,float stance_3,float stance_4,float swing){
    rclcpp::Rate step_time(1/time_to_wait);

    Eigen::Vector3d P3r(P3x, P3y, P3z);
    Eigen::Vector3d P3l(P3x, -P3y, P3z);
    
    compute_bezier_points(stance_1,swing, leg_vel[7], P3l, bezier_points,trajectory_points_1,true);  //lf
    compute_bezier_points(stance_2,swing, leg_vel[4], P3r, bezier_points,trajectory_points_2,false); //rf
    compute_bezier_points(stance_3,swing, leg_vel[6], P3l, bezier_points,trajectory_points_3,false); //lh
    compute_bezier_points(stance_4,swing, leg_vel[5], P3r, bezier_points,trajectory_points_4,true);  //rh


    for (size_t i = 0; i < trajectory_points_1.size(); ++i) {
        const auto& point1 = trajectory_points_1[i];
        const auto& point2 = trajectory_points_2[i];
        const auto& point3 = trajectory_points_3[i];
        const auto& point4 = trajectory_points_4[i];        

        auto request = std::make_shared<IKService::Request>();
        waypoints wp;

        // Leg-1

        request->x = point1.x;
        request->y = point1.y;
        request->z = point1.z;
        request->leg_no = 0;
        auto result1 = client_->async_send_request(request);
        result1.wait();

        auto response1 = result1.get();
        wp.theta1 = response1->theta1;
        wp.theta2 = response1->theta2;
        wp.theta3 = response1->theta3;



        // Leg-2
        request->x = point2.x;
        request->y = point2.y;
        request->z = point2.z;
        request->leg_no = 1;
        auto result2 = client_->async_send_request(request);
        result2.wait();

        auto response2 = result2.get();
        wp.theta4 = response2->theta1;
        wp.theta5 = response2->theta2;
        wp.theta6 = response2->theta3;

        // Leg-3
        request->x = point3.x;
        request->y = point3.y;
        request->z = point3.z;
        request->leg_no = 0;

        auto result3 = client_->async_send_request(request);
        result3.wait();

        auto response3 = result3.get();
        wp.theta7 = response3->theta1;
        wp.theta8 = response3->theta2;
        wp.theta9 = response3->theta3;

        // Leg-4
        request->x = point4.x;
        request->y = point4.y;
        request->z = point4.z;
        request->leg_no = 1;

        auto result4 = client_->async_send_request(request);
        result4.wait();

        auto response4 = result4.get();
        wp.theta10 = response4->theta1;
        wp.theta11 = response4->theta2;
        wp.theta12 = response4->theta3;

        // std::cout << "Leg1 values: "<< wp.theta1 << ", " << wp.theta2 << ", " << wp.theta3 << std::endl;
        // std::cout << "Leg2 values: "<< wp.theta4 << ", " << wp.theta5 << ", " << wp.theta6 << std::endl;
        // std::cout << "Leg3 values: "<< wp.theta7 << ", " << wp.theta8 << ", " << wp.theta9 << std::endl;
        // std::cout << "Leg4 values: "<< wp.theta10 << ", " << wp.theta11 << ", " << wp.theta12 << std::endl;
        send_trajectory_point(wp.theta1,wp.theta2,wp.theta3,wp.theta4,wp.theta5,wp.theta6,wp.theta7,wp.theta8,wp.theta9,wp.theta10,wp.theta11,wp.theta12,0.01);
        step_time.sleep();
        }
    }


    std::vector<traj_points> trajectory_points_1;
    std::vector<traj_points> trajectory_points_2;
    std::vector<traj_points> trajectory_points_3;
    std::vector<traj_points> trajectory_points_4;
    std::vector<double> leg_vel;

    double l =0.3; // width of robot
    double c = 0.3868;// lenght of robot

    float d=0.0, h=0.0, theta=0.0, P3x=+0.1934, P3y=-0.142, P3z=-0.25;
    int bezier_points=25;
    double cp_max=0.027, freq_max=1.66, freq_min=0.80; 
    double max_v = 0.16;

    float t_sec= 0.5;
    double time_to_wait = (t_sec / bezier_points);  // Convert to milliseconds
    float linear_vel=0.0;
    float ang_vel=0.0;
    

    rclcpp::Client<IKService>::SharedPtr client_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber;
    rclcpp::CallbackGroup::SharedPtr callback_group_client;


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControllerNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}



