//
// Created by yunfan on 2020/12/22.
//

#ifndef MAIN_CONTROLLER_MAV_FSM_H
#define MAIN_CONTROLLER_MAV_FSM_H
#include "main_controller/common_include.h"
namespace wtr{

    class MavFsmNode{
    private:
        ros::NodeHandle node_;
        ros::Subscriber vision_sub_,rc_sub_;
        ros::Publisher local_pos_pub_,speed_pub_;
        ros::Timer mav_fsm_;
        geometry_msgs::Pose init_pose_;
        geometry_msgs::PoseStamped takeoff_pose_;
        geometry_msgs::TwistStamped land_signal_;
        bool is_init_pose_;
        typedef enum{
            takeoff,
            attctr,
            land,
            waiting
        }workstate_t;
        workstate_t work_state_;
        /**
        std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
        uint8 rssi
        uint16[] channels
     * @param msg
     */

        void rcCallback(const mavros_msgs::RCInConstPtr &msg){

            if(msg->channels[7] > 1800){    // D开关向上 降落
                work_state_ = land;
            }else if(msg->channels[5]< 1500){ // 开关C向下
                work_state_=takeoff;
            }else if ( msg -> channels[5] > 1500){ // 开关C向上 使用姿态控制器
                if(work_state_ == takeoff)
                    work_state_ = attctr;
            }
        }

        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
            init_pose_ = msg->pose;
            is_init_pose_ = true;
            if( msg->pose.position.z>2.5)
            {
                work_state_ = land;
            }
        }

        void main_FSM(const ros::TimerEvent& /*event*/){
            static int cnt = 0;
            if(!is_init_pose_) work_state_ = waiting;
            switch (work_state_) {
                case waiting: {
                    if(cnt++ > 50)// make the information frequency reduce to 1Hz.
                    {
                        cnt=0;
                        ROS_INFO("[WTR MAV MAIN FSM][WAITING]:--------------Waiting for command.-------------------");
                    }
                    break;

                }
                case takeoff: {
                    if(cnt++ > 50)// make the information frequency reduce to 1Hz.
                    {
                        cnt=0;
                        ROS_INFO("[WTR MAV MAIN FSM][TAKEOFF]:--------------TAKEOFF COMMAND RECEIVED-------------------");
                    }
                    takeoff_pose_.header.stamp = ros::Time::now();
                    local_pos_pub_.publish(takeoff_pose_);
                    break;
                }
                case land: {
                    if(cnt++ > 50)// make the information frequency reduce to 1Hz.
                    {
                        cnt=0;
                        ROS_INFO("[WTR MAV MAIN FSM][LAND]:--------------LAND COMMAND RECEIVED-------------------");
                    }
                    land_signal_.header.stamp = ros::Time::now();
                    land_signal_.header.frame_id = "body";
                    speed_pub_.publish(land_signal_);
                    break;
                }
                case attctr: {
                    ROS_WARN("[WTR MAV MAIN FSM]:--------------Attitude mode has not been defined!--------------------");
                }
            }
            ros::spinOnce();
        }


    public:
        MavFsmNode(ros::NodeHandle & nh){
            node_ = nh;
            is_init_pose_ = false;
            work_state_ =  waiting;



            /*  Define the pulisher of point and velocity.  */
            local_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>
                    ("mavros/setpoint_position/local", 10);
            speed_pub_ = nh.advertise<geometry_msgs::TwistStamped>
                    ("/mavros/setpoint_velocity/cmd_vel", 10);
            /*  Define the subscriber of mocap system or realsense. */
            vision_sub_ = nh.subscribe
                    ("/mavros/vision_pose/pose", 10, &MavFsmNode::poseCallback, this);

            /*  Define the mavros RC callback.*/
            rc_sub_ = nh.subscribe
                    ("/mavros/rc/in", 10, &MavFsmNode::rcCallback, this);


            mav_fsm_ = node_.createTimer(ros::Duration(0.02), &MavFsmNode::main_FSM, this);
            ROS_INFO("INIT FSM TIMER SUCCESS!");

            /*  Waiting for the pose initialization  */
            while(!is_init_pose_){
                ros::spinOnce();
            }

            memset(&land_signal_, 0 ,sizeof(land_signal_));
            land_signal_.twist.linear.z = -0.4;
            land_signal_.twist.linear.x = 0;
            land_signal_.twist.linear.y = 0;
            takeoff_pose_.pose = init_pose_;
            takeoff_pose_.pose.position.z += 1.3;
            takeoff_pose_.header.frame_id = "mav";
            ROS_INFO("MAV INIT SUCCESS!");



        };

        ~MavFsmNode(){};
        typedef std::shared_ptr<MavFsmNode> Ptr;

    };


}



#endif //MAIN_CONTROLLER_MAV_FSM_H
