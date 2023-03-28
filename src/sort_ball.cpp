//
// Created by gcote2021 on 26/01/23.
//

#include "lab_tricolor/sort_ball.hpp"
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <memory>

namespace lab_tricolor {

    class LabNode : public SortBall {
        public:
            LabNode(NodeOptions opts) : SortBall(opts) {

            }
        protected:
            bool checkTSource();
            bool checkCentering();
            bool checkApproach();
            bool checkGrip();
            bool checkTDest();
            bool checkRelease();

            void actionTSource();
            void actionCentering();
            void actionApproach();
            void actionGrip();
            void actionTDest();
            void actionRelease();
    };

    /**
     * Check whether the robot is at the source point or not
     */
    bool LabNode::checkTSource() {
        return false;
    }
    /**
     * Compute the next action to do during the translation to the source
     */
    void LabNode::actionTSource() {

    }

    /**
     * Check whether the ball is centered on the camera image.
     */
    bool LabNode::checkCentering() {
        float thr = 0.01;
        bool check_xy = (abs(x_cam)<thr) && (abs(y_cam)<thr);
        return check_xy;
    }
    /**
     * Describe the centering action
     */
    void LabNode::actionCentering() {
        auto twist_mat = computeTwistCenter();
        //std::cout <<"twist mat built : " <<twist_mat<<std::endl;
        lab_tricolor::srv::Jacobian::Request req;
        req.inverse = true;
        req.ee_frame = true;
        if(state.position.size()){      //check for initialisation of state variable
            get_pos(state,req,side);
            if(lab_tricolor::srv::Jacobian::Response res; jac_node.call(req, res)){
                command.set__mode(command.VELOCITY_MODE);
                auto com = computeCommand(res.jacobian,twist_mat);
                command.set__command(com);
                command_ini = true;
            }
        }
    }
            
            

    /**
     * Check whether the robot has ended the approach phase
     */
    bool LabNode::checkApproach() {

        return false;
    }
    /**
     * Describe the Approach action
     */
    void LabNode::actionApproach() {
        // BaxterAction msg;
        // msg.component = side +"_gripper";
        // msg.action = msg.CMD_GRIP;
        // pub_gripper.publish(msg);
    }

    /**
     * Check whether the robot has gripped the ball or not
     */
    bool LabNode::checkGrip() {
        return false;
    }
    /**
     * Describe the gripping action
     */
    void LabNode::actionGrip() {
        // BaxterAction msg;
        // msg.component = side +"_gripper";
        // msg.action = msg.CMD_GRIP;
        // pub_gripper.publish(msg);
    }

    /**
     * Check whether the robot is at the destination point or not
     */
    bool LabNode::checkTDest() {
        return false;
    }
    /**
     * Describe the movement to the destination box
     */
    void LabNode::actionTDest() {
            //TO DO : index_point gives us the point we need to go to, and the list must be added to vec_points at ini
        geometry_msgs::msg::Point Point_to_get = vec_points[index_point];
            // build service request SolvePositionIK::Request from obtained transform
        baxter_core_msgs::srv::SolvePositionIK::Request req;
        std::vector<geometry_msgs::msg::PoseStamped> Poses;
        geometry_msgs::msg::PoseStamped PoseStamped;
            //TO DO : The orientation need to be defined.
        //PoseStamped.pose.orientation = ?; 
        PoseStamped.pose.position.x = Point_to_get.x;
        PoseStamped.pose.position.y = Point_to_get.y;
        PoseStamped.pose.position.z = Point_to_get.z;
        Poses.push_back(PoseStamped);
        req.set__seed_mode(req.SEED_AUTO);
        req.set__pose_stamp(Poses);
        if(baxter_core_msgs::srv::SolvePositionIK::Response res; ik_node.call(req, res)){
            // call to IK was successfull, check if the solution is valid
            if (res.is_valid[0]  ){
                command.names = res.joints[0].name;
                int n = res.joints[0].position.size();
                for (int k = 0; k<n;k++){
                    command.command.resize(7);
                    command.command[k] = res.joints[0].position[k];
                    command.set__mode(command.RAW_POSITION_MODE);
                }
                pub_command->publish(command);
            }
        }
    }

    /**
     * Check whether the robot has release the ball or not
     */
    bool LabNode::checkRelease() {
        return false;
    }
    /**
     * Describe the release action
     */
    void LabNode::actionRelease() {
        // BaxterAction msg;
        // msg.component = side +"_gripper";
        // msg.action = msg.CMD_RELEASE;
        // pub_gripper.publish(msg);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lab_tricolor::LabNode>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
}
