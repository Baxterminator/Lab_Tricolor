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