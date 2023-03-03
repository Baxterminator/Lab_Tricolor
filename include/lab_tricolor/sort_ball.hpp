//
// Created by gcote2021 on 26/01/23.
//

#ifndef BUILD_SORT_BALL_H
#define BUILD_SORT_BALL_H

#include "rclcpp/rclcpp.hpp"
#include "lab_tricolor/step_node.hpp"
//#include "lab_tricolor/eigen.h"
#include "lab_tricolor/ik_client.h"

#include "step_node.hpp"

#include "std_msgs/msg/float32_multi_array.hpp" //for circle
#include "sensor_msgs/msg/joint_state.h" //for state
#include "baxter_core_msgs/msg/joint_command.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "baxter_core_msgs/srv/solve_position_ik.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <lab_tricolor/srv/jacobian.hpp>
#include <eigen3/Eigen/Core>
#include <Eigen/QR>    


#include <map>
namespace lab_tricolor {

    using namespace rclcpp;


    enum class Step {
        IDLE,
        T_SOURCE,
        CENTERING,
        APPROACH,
        GRIP,
        T_DEST,
        RELEASE
    };

    const std::vector<std::string> suffixes = {"_s0", "_s1", "_e0", "_e1", "_w0", "_w1", "_w2"};

    class SortBall : public StepNode<Step> {
    public:
        explicit SortBall(NodeOptions opts) : StepNode<Step>("sort_ball", opts) {
            sub_circle = create_subscription<std_msgs::msg::Float32MultiArray>(
                topic_circle,    // which topic
                1,         // QoS : real time
                [this](std_msgs::msg::Float32MultiArray::UniquePtr msg)    // callback are perfect for lambdas
                {
                    circle = *msg;
            });
            sub_joint_state = create_subscription<sensor_msgs::msg::JointState>(
                topic_state,    // which topic
                1,         // QoS : real time
                [this](sensor_msgs::msg::JointState::UniquePtr msg)    // callback are perfect for lambdas
                {
                    state = *msg;
            });
            pub_command = create_publisher<baxter_core_msgs::msg::JointCommand>("robot/limb/"+side+"/joint_command", 1);   // topic + QoS
            timer = create_wall_timer(1000ms,    // rate
                                              [&](){if(command_ini){pub_command->publish(command);}});
            //initializing the service that compute the Jacobian (inverse, in this program)
            jac_node.init("jac_node","/robot/limb/" + side + "/jacobian");
            //create_client<lab_tricolor::srv::Jacobian>("/robot/limb/"+side+"/jacobian ");
            //jacobian_service_.init("jacobian_"+side,"/robot/limb/"+side+"/jacobian", 100ms); //timeout after 100ms
 
            // INITIALIZE COMMAND NAME / MODE : 
            command.set__mode(command.POSITION_MODE);
            std::vector<std::string> names(7);
            for(int i=0;i<7;i++)
                names[i] = side + suffixes[i];
            command.set__names(names);
        }


        inline Eigen::MatrixXd compute_Ls_inv(const double& x,const double& y,const double& z){
            std::vector<double> Ls_vector={-1/z,0,x/z,x*y,-(1+std::pow(x,2)),y,0,-1/z,y/z,1+std::pow(y,2),-x*y,-x};
            Eigen::Matrix<double,2,6, Eigen::RowMajor> Ls(Ls_vector) ;
            std::cout << "Matrix Ls :" << Ls <<std::endl;
            Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cqr(Ls);
            //Ls.data() = {-1/z,0,x/z,x*y,-(1+std::pow(x,2)),y,0,-1/z,y/z,1+std::pow(y,2),-x*y,-x};
            //std::copy(Ls_vector.begin(), Ls_vector.end(), Ls.data());
            auto Ls_inverse = cqr.pseudoInverse();
            return Ls_inverse;
        }
        inline std::vector<double> computeCommand(const std::array<double, 42> &Jinv_coeffs,
                                          const Eigen::Matrix<double,6,1> &vec_twist)
        {
            Eigen::Matrix<double,7,6, Eigen::RowMajor> Jinv;
            std::copy(Jinv_coeffs.begin(), Jinv_coeffs.end(), Jinv.data());
            const Eigen::Matrix<double,7,1> cmd{Jinv*vec_twist};
            return {cmd.data(), cmd.data()+7};
        }
        // void write_Twist_msg(const Eigen::Matrix<double,6,1> &eigen_twist,geometry_msgs::msg::Twist &twist){ 
        //     twist.linear.set__x(eigen_twist(0,0));
        //     twist.linear.set__y(eigen_twist(1,0));
        //     twist.linear.set__z(eigen_twist(2,0));
        //     twist.angular.set__x(eigen_twist(3,0));
        //     twist.angular.set__y(eigen_twist(4,0));
        //     twist.angular.set__z(eigen_twist(5,0));
        // }


    protected:
        /*
         * ################################################################
         *                          Step Management
         * ################################################################
         */
        Step check_step(Step act_step) override {
            if (act_step == Step::RELEASE)
                return Step::IDLE;
            return act_step;
        }

        /*
         * ################################################################
         *                          Step Matching
         * ################################################################
         */
        // Map of function to use for checking and acting at each step
        const std::map<Step, cfunc> check_map = {
                {Step::IDLE, [this] () { return checkIdle();}},
                {Step::T_SOURCE, [this] () { return checkTSource(); }},
                {Step::CENTERING, [this] () { return checkCentering(); }},
                {Step::APPROACH, [this] () { return checkApproach(); }},
                {Step::GRIP, [this] () { return checkGrip(); }},
                {Step::T_DEST, [this] () { return checkTDest(); }},
                {Step::RELEASE, [this] () { return checkRelease(); }},
        };
        const std::map<Step, afunc> action_map = {
                {Step::T_SOURCE, [this] () { return actionTSource();}},
                {Step::CENTERING, [this] () { return actionCentering();}},
                {Step::APPROACH, [this] () { return actionApproach();}},
                {Step::GRIP, [this] () { return actionGrip();}},
                {Step::T_DEST, [this] () { return actionTDest();}},
                {Step::RELEASE, [this] () { return actionRelease();}},
        };

        /*
         * ################################################################
         *                          Step Description
         * ################################################################
         */
        // TODO: To implement
        bool checkIdle() {
            return false;
        }
        virtual bool checkTSource() = 0;
        virtual bool checkCentering() = 0;
        virtual bool checkApproach() = 0;
        virtual bool checkGrip() = 0;
        virtual bool checkTDest() = 0;
        virtual bool checkRelease() = 0;

        virtual void actionTSource() = 0;
        virtual void actionCentering() = 0;
        virtual void actionApproach() = 0;
        virtual void actionGrip() = 0;
        virtual void actionTDest() = 0;
        virtual void actionRelease() = 0;

        std::string side = "left";

        Publisher<baxter_core_msgs::msg::JointCommand>::SharedPtr pub_command;
        baxter_core_msgs::msg::JointCommand command;
        bool command_ini=false;

        Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_circle;
        Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state;

        std_msgs::msg::Float32MultiArray circle;
        sensor_msgs::msg::JointState state;
        rclcpp::TimerBase::SharedPtr timer;
        std::string topic_circle;
        std::string topic_state = "/robot/joint_states";

        ServiceNodeSync<lab_tricolor::srv::Jacobian> jac_node;


        //ServiceNodeSync<baxter_simple_sim::srv::Jacobian> jacobian_service_;


    };

}

#endif //BUILD_SORT_BALL_H
