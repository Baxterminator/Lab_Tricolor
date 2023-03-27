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
            bool checkApproach();
            bool checkGrip();
            bool checkTDest();
            bool checkRelease();

            void actionTSource();
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
    bool LabNode::checkApproach() {
        return false;
    }
    /**
     * Describe the centering action
     */
    void LabNode::actionApproach() {
        RCLCPP_INFO_ONCE(this->get_logger(), "Approach");
        if(circle.data.size()){
            double x_grip = circle.data[0];
            double y_grip = circle.data[1];
            Eigen::Matrix<double,2,1> e = {
                x_grip-0,y_grip-0
            }; //à verif : peut etre 0,0 au lieu de centre en pixel
            RCLCPP_INFO_ONCE(this->get_logger(), "Made e (error)");
            double lambda = 1.2;
            double lambda_z = 0.05;
            double R_circle = pow((circle.data[2])/M_PI,0.5);
            double Dmesured = Zref * Rref/R_circle;
            double Zmesured = pow(pow(Dmesured,2)-pow(x_grip/10,2)-pow(y_grip/10,2),0.5);
            std::cout<<"x:" <<circle.data[0]<<" y:"<<circle.data[1]<<" area:"<<circle.data[2] <<std::endl;
            RCLCPP_INFO_ONCE(this->get_logger(), "Building Ls");
            if(abs(Zmesured)<0.00001){Zmesured=Zmesured*100;}
            std::cout<<"R_circle : "<<R_circle<<std::endl;
            std::cout<<"R_ref : "<<Rref<<std::endl;
            std::cout<<"D_mesured : "<<Dmesured<<std::endl;


            std::cout<<"Zmesured : "<<Zmesured<<std::endl;
            std::cout<<pow(Dmesured,2)<<" : "<<pow(x_grip/10,2)<<" : "<<pow(y_grip/10,2)<<std::endl;
            Eigen::MatrixXd Ls_inv = compute_Ls_inv(x_grip/Zmesured,y_grip/Zmesured,Zmesured);
            Eigen::Matrix<double,6,1> twist_mat = -lambda*Ls_inv*e;
            twist_mat(4)=0;
            twist_mat(5)=0;

            Eigen::Matrix<double,6,1> twist_mat_z{0,0,0,0, 0, 0}; //-lambda_z*(Zref-Zmesured)

            std::cout <<"twist mat built : " <<twist_mat<<std::endl;
            auto req = std::make_shared<lab_tricolor::srv::Jacobian::Request>();
            req->inverse = true;
            req->ee_frame = true;
            int start_array = 2;
            if(side=="right"){start_array = 9;} // state position give hea and torso position on 0,1 then left arm chain then right arm chain

            RCLCPP_INFO(this->get_logger(), "start_array initialised");
            if(state.position.size()){      //check for initialisation of state variable
                for (int i=0;i<7;i++){
                    req->position[i]= state.position[i+start_array];
                }
                RCLCPP_INFO(this->get_logger(), "req built");
                if(lab_tricolor::srv::Jacobian::Response res; jac_node.call(req, res)){
                    RCLCPP_INFO_ONCE(this->get_logger(), "Got response from jac_node");
                    command.set__mode(command.VELOCITY_MODE);
                    auto com = computeCommand(res.jacobian,twist_mat+twist_mat_z);
                    command.set__command(com);
                    command_ini = true;
                }
            }
            
        }
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
