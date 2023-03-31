#ifndef BUILD_SORT_BALL_H
#define BUILD_SORT_BALL_H


#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <iterator>
#include <vector>


#include <geometry_msgs/msg/twist.hpp>
#include "std_msgs/msg/float32_multi_array.hpp" //for circle
#include "sensor_msgs/msg/joint_state.h" //for state
#include <baxter_core_msgs/msg/joint_command.hpp>
#include "baxter_core_msgs/srv/solve_position_ik.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <lab_sort/utilities.hpp>
#include <lab_sort/srv/jacobian.hpp>
#include "lab_sort/step_node.hpp"
#include "lab_sort/ik_client.h"

//#include "ecn_baxter/msg/BaxterAction.msg" //to make gripping easier

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2/LinearMath/Quaternion.h>

#include <map>




namespace lab_sort {

    using namespace rclcpp;
    //using ecn_baxter::msg::BaxterAction;


    enum Step {
        IDLE,
        T_SOURCE,
        CENTERING,
        GRIP,
        T_DEST,
        RELEASE
    };

    const std::vector<std::string> suffixes = {"_s0", "_s1", "_e0", "_e1", "_w0", "_w1", "_w2"};

    class LabSort : public StepNode<Step> {
    public:
        explicit LabSort(NodeOptions opts) : StepNode<Step>("lab_sort", opts) {

            check_map = {
                {Step::IDLE, [this] () { return checkIdle();}},
                {Step::T_SOURCE, [this] () { return checkTSource(); }},
                {Step::CENTERING, [this] () { return checkCentering(); }},
                {Step::GRIP, [this] () { return checkGrip(); }},
                {Step::T_DEST, [this] () { return checkTDest(); }},
                {Step::RELEASE, [this] () { return checkRelease(); }},
            };

            action_map = {
                {Step::T_SOURCE, [this] () { return actionTSource();}},
                {Step::CENTERING, [this] () { return actionCentering();}},
                {Step::GRIP, [this] () { return actionGrip();}},
                {Step::T_DEST, [this] () { return actionTDest();}},
                {Step::RELEASE, [this] () { return actionRelease();}},
            };

            this->declare_parameter("side", "left");
            side = this->get_parameter("side").get_parameter_value().get<std::string>();
            
            topic_circle = "/robot/"+side+"_circle";

            // To get to centering fast (debug only)
            increment_step();
            increment_step();

            sub_circle = create_subscription<std_msgs::msg::Float32MultiArray>(
                topic_circle,    // which topic
                1,         // QoS : real time
                [this](std_msgs::msg::Float32MultiArray::UniquePtr msg)    // callback are perfect for lambdas
                {
                    circle = *msg;
                    x_cam   = circle.data[0];
                    y_cam   = circle.data[1];
                    area    = circle.data[2];
                });
            sub_joint_state = create_subscription<sensor_msgs::msg::JointState>(
                topic_state,    // which topic
                1,         // QoS : real time
                [this](sensor_msgs::msg::JointState::UniquePtr msg)    // callback are perfect for lambdas
                {
                    state = *msg;
            });
            pub_command = create_publisher<baxter_core_msgs::msg::JointCommand>("robot/limb/"+side+"/joint_command", 1);   // topic + QoS
            timer = create_wall_timer(100ms,    // rate
                                              [&](){if(command_ini){pub_command->publish(command);}});
            //pub_gripper = create_publisher<BaxterAction>(topic_gripper, 1);
            
            // initializing the service that compute the Jacobian (inverse, in this program) and the inverse kinematic (ik)
            jac_node.init("jac_node","/robot/" + side + "/jacobian");
            ik_node.init("ik_node","/ExternalTools/" + side + "/PositionKinematicsNode/IKService");
 
            // Initialize command name / mode : 
            command.set__mode(command.POSITION_MODE);
            std::vector<std::string> names(7);
            for(int i=0;i<7;i++)
                names[i] = side + suffixes[i];
            command.set__names(names);

            // Initialize the tf2 listener :
            tf_buffer =
                std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener =
                std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
            RCLCPP_INFO_ONCE(this->get_logger(), "End of ini");

        }

    protected:
        /*
         * ################################################################
         *                          Step Management
         * ################################################################
         */
        Step check_step(Step act_step) override {
            if (act_step == Step::RELEASE)
                return Step::IDLE;
            if (act_step == Step::IDLE)
                return Step::T_SOURCE;
            if (act_step == Step::T_SOURCE)
                return Step::CENTERING;
            return act_step;
        }

        /*
         * ################################################################
         *                          Step Matching
         * ################################################################
         */
        // Map of function to use for checking and acting at each 

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
        virtual void actionAprroach() = 0;
        virtual void actionGrip() = 0;
        virtual void actionTDest() = 0;
        virtual void actionRelease() = 0;

        std::string side = "left";

        Publisher<baxter_core_msgs::msg::JointCommand>::SharedPtr pub_command;
        baxter_core_msgs::msg::JointCommand command;
        bool command_ini=false;

        Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_circle;
        Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state;

        float x_cam;
        float y_cam;
        float area;

        std_msgs::msg::Float32MultiArray circle;
        sensor_msgs::msg::JointState state;
        rclcpp::TimerBase::SharedPtr timer;
        std::string topic_circle = "/robot/"+side+"_circle";
        const std::string topic_state = "/robot/joint_states";

        //Publisher<BaxterAction>::SharedPtr pub_gripper;
        std::string topic_gripper = "/baxter/action";

        ServiceNodeSync<lab_sort::srv::Jacobian> jac_node;
        ServiceNodeSync<baxter_core_msgs::srv::SolvePositionIK> ik_node;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr}; // subscribes to /tf
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;              // stores all previous elementary transforms in a tree

        std::vector<geometry_msgs::msg::Point> vec_points;
        std::vector<std::string> vec_name_points;
        int index_point=0;

        constexpr double static Zref = 0.14 *2; //0.14
        const double Rref = pow(0.052545081824064255/M_PI,0.5)/2;

        /*
         * ################################################################
         *                   Some Utility Function
         * ################################################################
         */

        inline Eigen::Matrix<double,6,1> computeTwistCenter(double lambda_cent=1.5,double lambda_hei=0.5,double lambda_rot=0.5,
            double x_center=0, double y_center=0, double height=Zref){
            Eigen::Matrix<double,2,1> e_angle = getAngleError();

            if(circle.data.size()){
                Eigen::Matrix<double,2,1> e = {x_cam-x_center,y_cam-y_center};
                double R_circle = pow((circle.data[2])/M_PI,0.5);
                double Dmesured = height * (Rref*(height/Zref))/R_circle;
                double Zmesured = pow(pow(Dmesured,2)-pow(x_cam/10,2)-pow(y_cam/10,2),0.5);
                //std::cout<<"x:" <<circle.data[0]<<" y:"<<circle.data[1]<<" area:"<<circle.data[2] <<std::endl;
                if(abs(circle.data[2])>0.00001){
                    Eigen::MatrixXd Ls_inv = compute_Ls_inv(x_cam/Zmesured,y_cam/Zmesured,Zmesured);
                    Eigen::Matrix<double,6,1> twist_mat = -lambda_cent*Ls_inv*e;
                    // We only 
                    twist_mat(2)=0;
                    twist_mat(4)=0;
                    twist_mat(5)=0;
                    // Apply the transform from cam to end-effector frame for Vx and Vy. (x_ee = y_cam, y_ee = x_cam)
                    auto temp = twist_mat(0);
                    twist_mat(0)=twist_mat(1);
                    twist_mat(1)=-temp;
                    // Compute the desired velocity along z :
                    twist_mat(2)=std::min(-lambda_hei*(Zref-Zmesured),1.);

                    twist_mat(4) = lambda_rot*e_angle(1);
                    twist_mat(5) = -lambda_rot*e_angle(0);
                    return twist_mat;
                }
            }
            return {0,0,0,0,0,0};
        }

        Eigen::Matrix<double,2,1> getAngleError(){
            Eigen::Matrix<double,2,1> e_angle{0,0};
            // All of this depend on the tf2 transform
            if(tf_buffer->canTransform(side + "_gripper", "base",
                tf2::TimePointZero, tf2::durationFromSec(1.0))){
                auto TransformStamped = tf_buffer->lookupTransform("base", side + "_gripper",
                    tf2::TimePointZero,tf2::durationFromSec(1.0)); 
                // TransformStamped has quaternion angles.
                tf2::Quaternion quater{TransformStamped.transform.rotation.x,
                TransformStamped.transform.rotation.y,
                TransformStamped.transform.rotation.z,
                TransformStamped.transform.rotation.w};
                quater.normalize();
                // We use the Matrix3x3 class to extract Roll-Pitch-Yaw angles from the quaternions.
                tf2::Matrix3x3 m(quater);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw); //yaw (rotation along z) is not useful in our case
                // We want to be stable at roll = +-pi and pitch = 0 (and roll is between -pi and pi)
                if(roll>0){ e_angle ={roll-M_PI,pitch}; }
                else{e_angle ={roll+M_PI,pitch};}
            }
            return e_angle;
        }
    };
}

#endif //BUILD_SORT_BALL_H
