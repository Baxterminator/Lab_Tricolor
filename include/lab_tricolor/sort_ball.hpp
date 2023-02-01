//
// Created by gcote2021 on 26/01/23.
//

#ifndef BUILD_SORT_BALL_H
#define BUILD_SORT_BALL_H

#include "rclcpp/rclcpp.hpp"
#include "ecn_baxter/game/step_node.hpp"
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

    class SortBall : public ECNBaxter::StepNode<Step> {
    public:
        explicit SortBall(NodeOptions opts) : StepNode<Step>("sort_ball", opts) {

        }
    protected:
        /*
         * ################################################################
         *                          Step Management
         * ################################################################
         */
        Step check_step(Step act_step) {
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
        const std::map<Step, ECNBaxter::cfunc> check_map = {
                {Step::IDLE, [this] () { return checkIdle();}},
                {Step::T_SOURCE, [this] () { return checkTSource(); }},
                {Step::CENTERING, [this] () { return checkCentering(); }},
                {Step::APPROACH, [this] () { return checkApproach(); }},
                {Step::GRIP, [this] () { return checkGrip(); }},
                {Step::T_DEST, [this] () { return checkTDest(); }},
                {Step::RELEASE, [this] () { return checkRelease(); }},
        };
        const std::map<Step, ECNBaxter::afunc> action_map = {
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
    };

}

#endif //BUILD_SORT_BALL_H
