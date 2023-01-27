//
// Created by gcote2021 on 26/01/23.
//

#ifndef BUILD_SORT_BALL_H
#define BUILD_SORT_BALL_H

#include "rclcpp/rclcpp.hpp"
#include "lab_tricolor/step_node.hpp"
#include "step_node.hpp"

#include <map>
namespace lab_tricolor {

    using namespace rclcpp;

    enum Step {
        IDLE,
        T_SOURCE,
        CENTERING,
        APPROACH,
        GRIP,
        T_DEST,
        RELEASE
    };

    class SortBall : public StepNode<Step> {
    public:
        explicit SortBall(NodeOptions opts) : StepNode<Step>("sort_ball", opts) {

        }
    protected:
        void init();
        /*
         * ################################################################
         *                          Step Management
         * ################################################################
         */
        Step check_step(int n_step) override {
            if (n_step > RELEASE)
                return IDLE;
            return static_cast<Step>(n_step);
        }

        /*
         * ################################################################
         *                          Step Matching
         * ################################################################
         */
        // Map of function to use for checking and acting at each step
        const std::map<Step, cfunc> check_map = {
                {IDLE, [this] () { return checkIdle();}},
                {T_SOURCE, [this] () { return checkTSource(); }},
                {CENTERING, [this] () { return checkCentering(); }},
                {APPROACH, [this] () { return checkApproach(); }},
                {GRIP, [this] () { return checkGrip(); }},
                {T_DEST, [this] () { return checkTDest(); }},
                {RELEASE, [this] () { return checkRelease(); }},
        };
        const std::map<Step, afunc> action_map = {
                {T_SOURCE, [this] () { return actionTSource();}},
                {CENTERING, [this] () { return actionCentering();}},
                {APPROACH, [this] () { return actionApproach();}},
                {GRIP, [this] () { return actionGrip();}},
                {T_DEST, [this] () { return actionTDest();}},
                {RELEASE, [this] () { return actionRelease();}},
        };

        /*
         * ################################################################
         *                          Step Description
         * ################################################################
         */
        // TODO: To implement
        bool checkIdle() {

        }
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

};

#endif //BUILD_SORT_BALL_H
