#include <fstream>
#include <streambuf>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <array>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <memory>
#include <iostream>
#include <Eigen/Core>
#include "quadrupled_control/srv/ik_service.hpp"


using std::placeholders::_1;

class LegIK : public rclcpp::Node {
public:
  LegIK() : Node("leg_ik_solver") {

    ik_service_ = this->create_service<quadrupled_control::srv::IKService>(
      "solve_ik",
      std::bind(&LegIK::IKCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    urdf_parser();


  }

private:

  void IKCallback(
    const std::shared_ptr<quadrupled_control::srv::IKService::Request> request,
    std::shared_ptr<quadrupled_control::srv::IKService::Response> response){

    double hip_angle = 0.0, up_angle=0.0 , knee_angle = 0.0;
    getJointAngles(request->x,request->y,request->z,request->leg_no,hip_angle,up_angle, knee_angle,response->success);


    response->theta1=hip_angle;
    response->theta2=up_angle;
    response->theta3=knee_angle;

    if (response->success == false){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Request failed , Aborting!!");

    }
    else{
       RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response Sent: [%f %f %f]",response->theta1,response->theta2,response->theta3);

        }

    }   



  void urdf_parser() {
    RCLCPP_INFO(this->get_logger(), "Received URDF, parsing...");
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("go2_description");
    std::string urdf_path = package_share_dir + "/xacro/leg_ik.xacro";
    std::string urdf;

    std::string command = "xacro " + urdf_path;
    std::array<char, 128> buffer;
    std::shared_ptr<FILE> pipe(popen(command.c_str(), "r"), pclose);
    
    if (!pipe) {
        throw std::runtime_error("Failed to run xacro command");
    }

    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        urdf += buffer.data();
    }


    if (!kdl_parser::treeFromString(urdf, tree_r)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF into KDL tree");
      return;
    }


    if (!tree_r.getChain("base_link", "rf_foot_link", chain_r)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to extract chain from base_link to right_end_link");
      return;
    }

   
    if (!kdl_parser::treeFromString(urdf, tree_l)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF into KDL tree");
      return;
    }


    if (!tree_l.getChain("base_link", "lf_foot_link", chain_l)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to extract chain from base_link to left_end_link");
      return;
    }

   
    Eigen::Matrix<double, 6, 1> weights;
    weights << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;  // Position only
    solver_r = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_r, weights,1E-5, 10000);
    solver_l = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_l, weights,1E-5, 10000);

    std::cout << "Chain extracted successfully!" << std::endl;

  }




  void getJointAngles(float x,float y,float z,int leg_no,double & hip_angle, double & up_angle, double & knee_angle,bool & success) {
    KDL::Frame p_in(KDL::Vector(x, y, z));
    KDL::JntArray q_out(chain_r.getNrOfJoints());

    if (leg_no==1){
      if (!solver_r) {
        RCLCPP_ERROR(this->get_logger(), "IK solver not initialized!");
        success=false;
        return;
      }


      // Initial guess
      KDL::JntArray q_init(chain_r.getNrOfJoints());
      q_init(0) = 0.0;
      q_init(1) = 1.014;
      q_init(2) = -2.02;

      int result = solver_r->CartToJnt(q_init, p_in, q_out);
      if (result < 0) {
          RCLCPP_ERROR(this->get_logger(), 
                      "IK failed with code %d. Unable to solve for position x: %.3f, y: %.3f, z: %.3f", 
                      result, x, y, z);
      }
      else{success=true;}
    }

    else{
            if (!solver_l) {
        RCLCPP_ERROR(this->get_logger(), "IK solver not initialized!");
        success=false;
        return;
      }


      // Initial guess
      KDL::JntArray q_init(chain_l.getNrOfJoints());
      q_init(0) = 0.0;
      q_init(1) = 1.014;
      q_init(2) = -2.02;

      int result = solver_l->CartToJnt(q_init, p_in, q_out);
      if (result < 0) {
          RCLCPP_ERROR(this->get_logger(), 
                      "IK failed with code %d. Unable to solve for position x: %.3f, y: %.3f, z: %.3f", 
                      result, x, y, z);
      }
      else{success=true;}
    }
    // Output joint values
    hip_angle = q_out(0);
    up_angle = q_out(1);
    knee_angle = q_out(2);
  }

    rclcpp::Service<quadrupled_control::srv::IKService>::SharedPtr ik_service_;

    KDL::Tree tree_l;
    KDL::Tree tree_r;
    KDL::Chain chain_l;
    KDL::Chain chain_r;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> solver_l;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> solver_r;


};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegIK>());
  rclcpp::shutdown();
  return 0;
}
