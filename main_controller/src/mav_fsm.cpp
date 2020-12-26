//
// Created by yunfan on 2020/12/22.
//
#include "main_controller/mav_fsm.h"
using namespace wtr;

void MavFsmNode::att_mode() {
    if(cnt++ > 50)// make the information frequency reduce to 1Hz.
    {
        cnt=0;
        ROS_WARN("[WTR MAV MAIN FSM][ATT]:--------------ATT HAS NOT BEEEN DEFINED-------------------");
    }
    /*  Write your own code here!!! */


}

void MavFsmNode::takeoff_mode() {
    if(cnt++ > 50)// make the information frequency reduce to 1Hz.
    {
        cnt=0;
        ROS_INFO("[WTR MAV MAIN FSM][TAKEOFF]:--------------TAKEOFF COMMAND RECEIVED-------------------");
    }
    takeoff_pose_.header.stamp = ros::Time::now();
    local_pos_pub_.publish(takeoff_pose_);
}

void MavFsmNode::land_mode() {
    if(cnt++ > 50)// make the information frequency reduce to 1Hz.
    {
        cnt=0;
        ROS_INFO("[WTR MAV MAIN FSM][LAND]:--------------LAND COMMAND RECEIVED-------------------");
    }
    land_signal_.header.stamp = ros::Time::now();
    land_signal_.header.frame_id = "body";
    speed_pub_.publish(land_signal_);

}

void MavFsmNode::wait_mode() {
    if(cnt++ > 50)// make the information frequency reduce to 1Hz.
    {
        cnt=0;
        ROS_INFO("[WTR MAV MAIN FSM][WAITING]:--------------Waiting for command.-------------------");
    }
}

void MavFsmNode::main_FSM(const ros::TimerEvent &) {
    if(!is_init_pose_) work_state_ = waiting;
    switch (work_state_) {
        case waiting: {
            wait_mode();break;
        }
        case takeoff: {
            takeoff_mode();break;
        }
        case land: {
            land_mode();break;
        }
        case attctr: {
            att_mode();break;
        }
        default:{
            wait_mode();
        }
    }
    ros::spinOnce();
}

void MavFsmNode::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if(is_init_pose_) return;
    init_pose_ = msg->pose;
    is_init_pose_ = true;
    if( msg->pose.position.z>2.5)
    {
    work_state_ = land;
    }
}

void MavFsmNode::rcCallback(const mavros_msgs::RCInConstPtr &msg) {

    if(msg->channels[7] > 1800){    // D开关向上 降落
        work_state_ = land;
    }else if(msg->channels[5]< 1500){ // 开关C向下
        work_state_=takeoff;
    }else if ( msg -> channels[5] > 1500){ // 开关C向上 使用姿态控制器
        if(work_state_ == takeoff)
            work_state_ = attctr;
    }
}



MavFsmNode::MavFsmNode(ros::NodeHandle &nh) {
    node_ = nh;
    is_init_pose_ = false;
    work_state_ =  waiting;

    /*  Define the pulisher of point and velocity.  */
    local_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
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
    takeoff_pose_.header.frame_id = "body";
    ROS_INFO("MAV INIT SUCCESS!");

}