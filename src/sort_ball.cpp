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
        static int x_center = 320 ; //x_offset: 320 -> 640/2
        static int y_center = 200;  //y_offset: 200 -> 400/2
        static float Zref = 0;
        static float Rref = 0;
        static float pi = 3.1415926;
        std::vector<float> e = {circle.data[0]-x_center,circle.data[1]-y_center}; //à verif : peut etre 0,0 au lieu de centre en pixel
        float lambda = 1;
        float R_circle = pow((circle.data[2]/pi),0.5);
        float Zmesured = Zref + Rref/R_circle;
        std::vector<std::vector<float>> Ls_inv(6);
        //Ls_inv.resize(6);
        for (int i=0;i<Ls_inv.size();i++){
            Ls_inv[i].resize(2);
        }
        geometry_msgs::msg::Twist Twist;

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
