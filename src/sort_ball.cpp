//
// Created by gcote2021 on 26/01/23.
//

#include "lab_tricolor/sort_ball.hpp"

namespace lab_tricolor {

    /**
     * Initialize everything you need from the start here
     */
    void SortBall::init() {

    }

    /**
     * Check whether the robot is at the source point or not
     */
    bool SortBall::checkTSource() {
        return false;
    }

    /**
     * Check whether the ball is centered on the camera image
     */
    bool SortBall::checkCentering() {
        return false;
    }

    /**
     * Check whether the robot is at the next point for the approaching phase
     */
    bool SortBall::checkApproach() {
        return false;
    }

    /**
     * Check whether the robot has gripped the ball or not
     */
    bool SortBall::checkGrip() {
        return false;
    }

    /**
     * Check whether the robot is at the destination point or not
     */
    bool SortBall::checkTDest() {
        return false;
    }

    /**
     * Check whether the robot has release the ball or not
     */
    bool SortBall::checkRelease() {
        return false;
    }

    /**
     * Compute the next action to do during the translation to the source
     */
    void SortBall::actionTSource() {

    }

    /**
     * Describe the centering action
     */
    void SortBall::actionCentering() {

    }

    /**
     * Describe the approach to the ball
     */
    void SortBall::actionApproach() {

    }

    /**
     * Describe the gripping action
     */
    void SortBall::actionGrip() {

    }

    /**
     * Describe the movement to the destination box
     */
    void SortBall::actionTDest() {

    }

    /**
     * Describe the release action
     */
    void SortBall::actionRelease() {

    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lab_tricolor::SortBall>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
}