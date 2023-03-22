//
// Created by gcote2021 on 26/01/23.
// Modified by B on 09/03/23.
//

#include "lab_tricolor/sort_ball.hpp"
#include <geometry_msgs/msg/detail/point__struct.hpp>

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
     * Check whether the ball is centered on the camera image
     */
    bool LabNode::checkCentering() {
        return false;
    }
    /**
     * Describe the centering action
     */
    void LabNode::actionCentering() {
        RCLCPP_INFO_ONCE(this->get_logger(), "Centering");
        Eigen::Matrix<double,2,1> e = {
            circle.data[0]-x_center,circle.data[1]-y_center
        }; //à verif : peut etre 0,0 au lieu de centre en pixel
        float lambda = 1;
        float R_circle = pow((circle.data[2]/pi),0.5);
        float Zmesured = Zref * Rref/R_circle;
        Eigen::MatrixXd Ls_inv = compute_Ls_inv(circle.data[0],circle.data[1],Zmesured);
        Eigen::Matrix<double,6,1> twist_mat = -lambda*Ls_inv*e;
        //Twist.angular.
        auto req = std::make_shared<lab_tricolor::srv::Jacobian::Request>();
        req->inverse = true;
        req->ee_frame = true;
        int start_array = 2;
        if(side=="right"){
            start_array = 9;
            //std::copy(state.position.begin()+9,state.position.begin()+16,req->position);
        }
        else{//std::copy(state.position.begin()+2,state.position.begin()+9,req->position);
        }
        for (int i=0;i<7;i++){
            req->position[i]= state.position[i+start_array];
        }
        if(lab_tricolor::srv::Jacobian::Response res; jac_node.call(req, res)){
            RCLCPP_INFO_ONCE(this->get_logger(), "Got response from jac_node");
            command.set__command(computeCommand(res.jacobian,twist_mat));
            command_ini = true;
        }
    }

    /**
     * Check whether the robot is at the next point for the approaching phase
     */
    bool LabNode::checkApproach() {
        return false;
    }
    /**
     * Describe the approach to the ball
     */
    void LabNode::actionApproach() {
        
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
        geometry_msgs::msg::Point Point_to_get = vec_points[index_point];
            // build service request SolvePositionIK::Request from obtained transform
        baxter_core_msgs::srv::SolvePositionIK::Request req;
        std::vector<geometry_msgs::msg::PoseStamped> Poses;
        geometry_msgs::msg::PoseStamped PoseStamped;
        PoseStamped.pose.orientation; //TO DO : Z vers le bas ?
        PoseStamped.pose.position.x = Point_to_get.x;
        PoseStamped.pose.position.y = Point_to_get.y;
        PoseStamped.pose.position.z = Point_to_get.z;
        Poses.push_back(PoseStamped);
        req.set__seed_mode(req.SEED_AUTO);
        req.set__pose_stamp(Poses);
        if(baxter_core_msgs::srv::SolvePositionIK::Response res; ik_node.call(req, res))
        {
            // call to IK was successfull, check if the solution is valid
            if (res.is_valid[0]  ){
                command.names = res.joints[0].name;
                int n = res.joints[0].position.size();
                for (int k = 0; k<n;k++){
                    command.command.resize(7);
                    command.command[k] = res.joints[0].position[k];
                    command.set__mode(command.RAW_POSITION_MODE);
                }
                RCLCPP_INFO(this->get_logger(), "Publishing");
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

    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lab_tricolor::LabNode>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
}
