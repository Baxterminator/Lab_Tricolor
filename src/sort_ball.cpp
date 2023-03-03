//
// Created by gcote2021 on 26/01/23.
//

#include "lab_tricolor/sort_ball.hpp"

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
        Eigen::Matrix<double,2,1> e = {circle.data[0]-x_center,circle.data[1]-y_center}; //à verif : peut etre 0,0 au lieu de centre en pixel
        float lambda = 1;
        float R_circle = pow((circle.data[2]/pi),0.5);
        float Zmesured = Zref * Rref/R_circle;
        auto Ls_inv = compute_Ls_inv(circle.data[0],circle.data[1],Zmesured);
        Eigen::MatrixXd twist_mat;
        Eigen::Matrix<double,6,1> twist_mat = -lambda*Ls_inv*e;
        //geometry_msgs::msg::Twist Twist;
        //Twist.angular.
        auto req = std::make_shared<lab_tricolor::srv::Jacobian::Request>();
        req->inverse = true;
        req->ee_frame = true;
        if(side=="left"){
            std::copy(state.position.begin()+2,state.position.begin()+9,req->position);
        }
        else{
            std::copy(state.position.begin()+9,state.position.begin()+16,req->position);
        }
        if(lab_tricolor::srv::Jacobian::Response res; jac_node.call(req, res)){
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
