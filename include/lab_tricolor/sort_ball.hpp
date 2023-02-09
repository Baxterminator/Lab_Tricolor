//
// Created by gcote2021 on 26/01/23.
//

#ifndef BUILD_SORT_BALL_H
#define BUILD_SORT_BALL_H

#include "rclcpp/rclcpp.hpp"
#include "lab_tricolor/step_node.hpp"
#include "lab_tricolor/eigen.h"
#include "lab_tricolor/ik_client.h"

#include "step_node.hpp"

#include "std_msgs/msg/float32_multi_array.hpp" //for circle
#include "baxter_core_msgs/msg/joint_command.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "baxter_core_msgs/srv/solve_position_ik.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <baxter_simple_sim/srv/jacobian.hpp>


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
            pub_command = create_publisher<baxter_core_msgs::msg::JointCommand>("robot/limb/"+side+"/joint_command", 1);   // topic + QoS
            timer = create_wall_timer(1000ms,    // rate
                                              [&](){/*Publish function*/;});
            //initializing the service that compute the Jacobian (inverse, in this program)
            jacobian_service_.init("jacobian_"+side,"/robot/limb/"+side+"/jacobian", 100ms); //timeout after 100ms
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

        //tf2_ros::Buffer tf_buffer;
        //tf2_ros::TransformListener tf_listener;

        Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_circle;
        std_msgs::msg::Float32MultiArray circle;
        rclcpp::TimerBase::SharedPtr timer;
        std::string topic_circle;



        ServiceNodeSync<baxter_simple_sim::srv::Jacobian> jacobian_service_;


    };

}

#endif //BUILD_SORT_BALL_H
