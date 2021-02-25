//
// Created by yunfan on 2020/12/22.
//
#include "main_controller/mav_fsm.h"
using namespace wtr;
using namespace std;

void MavFsmNode::att_mode() {
    if(cnt++ > 50)// make the information frequency reduce to 1Hz.
    {
        cnt=0;
        ROS_INFO("[WTR MAV MAIN FSM][ATT]:-------------- \033[33m ATT CONTROL WORKING \033[0m-------------------");
    }
    desired.cPosition = Vec3(0,0,0);
    desired.cPositionD.setZero();
    desired.cPositionDD.setZero();
    desired.yawD = 0;
    desired.yaw = 0;
    callSO3ControlOnce();
}

void MavFsmNode::takeoff_mode() {
    if(cnt++ > 50)// make the information frequency reduce to 1Hz.
    {
        cnt=0;
        ROS_INFO("[WTR MAV MAIN FSM][TAKEOFF]:--------------\033[36m TAKEOFF \033[0m-------------------");
    }
    takeoff_pose_.header.stamp = ros::Time::now();
    local_pos_pub_.publish(takeoff_pose_);
}

void MavFsmNode::land_mode() {
    if(cnt++ > 50)// make the information frequency reduce to 1Hz.
    {
        cnt=0;
        ROS_INFO("[WTR MAV MAIN FSM][LAND]:--------------\033[32m LANDING \033[0m-------------------");
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

//    if(msg->channels[6] > 1100){    // C开关向上 降落
//        work_state_ = land;
//    }else if(msg->channels[5]< 1500){ // 开关B向上
//        work_state_=takeoff;
//    }else if ( msg -> channels[5] > 1500){ // 开关B向下 使用姿态控制器
//        if(work_state_ == takeoff)
//            work_state_ = attctr;
//    }


    /* WFT09II遥控器 */
    if(msg->channels[4] > 1500){    // C开关向上 降落
        work_state_ = land;
    }else if(msg->channels[5]< 1500){ // 开关B向上
        work_state_=takeoff;
    }else if ( msg -> channels[5] > 1500){ // 开关B向下 使用姿态控制器
        if(work_state_ == takeoff)
            work_state_ = attctr;
    }
}

void MavFsmNode::imuCallback(const sensor_msgs::ImuConstPtr &imu_msg) {

    feedback.cPositionDD = Vec3(imu_msg->linear_acceleration.x,
                                imu_msg->linear_acceleration.y,
                                imu_msg->linear_acceleration.z);

    controller_.setAcc(feedback.cPositionDD);
}

void MavFsmNode::odomCallback(const nav_msgs::OdometryConstPtr &odom_msg) {
    feedback.cPositionD = Vec3(
            odom_msg->twist.twist.linear.x,
            odom_msg->twist.twist.linear.y,
            odom_msg->twist.twist.linear.z
    );
    feedback.cPosition = Vec3(
            odom_msg->pose.pose.position.x,
            odom_msg->pose.pose.position.y,
            odom_msg->pose.pose.position.z
    );
    controller_.setPosition( feedback.cPosition);
    controller_.setVelocity( feedback.cPositionD);

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
    att_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>
            ("/mavros/setpoint_raw/attitude",10);
    desi_att_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/desired/att",10);
    /*  Define the subscriber of mocap system or realsense. */
    vision_sub_ = nh.subscribe
            ("/mavros/vision_pose/pose", 10, &MavFsmNode::poseCallback, this);
    odom_sub_ = nh.subscribe(
            "/mavros/local_position/odom",10,&MavFsmNode::odomCallback,this);
    imu_sub_ = nh.subscribe(
            "/mavros/imu/data",10,&MavFsmNode::imuCallback,this);
    /*  Define the mavros RC callback.*/
    rc_sub_ = nh.subscribe
            ("/mavros/rc/in", 10, &MavFsmNode::rcCallback, this);


    mav_fsm_ = node_.createTimer(ros::Duration(0.02), &MavFsmNode::main_FSM, this);
    ROS_INFO("INIT FSM TIMER SUCCESS!");

    /*  Get the parameters */
    nh.param<double>("/k_position/x",fp.k_position.x(),0.0);
    nh.param<double>("/k_position/y",fp.k_position.y(),0.0);
    nh.param<double>("/k_position/z",fp.k_position.z(),0.0);

    nh.param<double>("/k_velocity/x",fp.k_velocity.x(),0.0);
    nh.param<double>("/k_velocity/y",fp.k_velocity.y(),0.0);
    nh.param<double>("/k_velocity/z",fp.k_velocity.z(),0.0);

    controller_.setMass(2.0);
    controller_.setGravity(cal_.g);

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

void MavFsmNode::updateOutputOrientation() {
    output.orientation.x = output.quad.x();
    output.orientation.y = output.quad.y();
    output.orientation.z = output.quad.z();
    output.orientation.w = output.quad.w();
}

void MavFsmNode::callSO3ControlOnce() {
    now_ = ros::Time::now();
    const double dt = cal_.getDt(ros::Time::now().toSec());
    if(fp.k_position.norm()<1e-3){
        ROS_WARN("PID parameters has not been set, force return!");
        return;
    }
    controller_.calculateControl(desired.cPosition, desired.cPositionD, desired.cPositionDD,
                                 desired.yaw,desired.yawD,
                                 fp.k_position,fp.k_velocity);
    output.force = controller_.getComputedForce().norm();
    output.orientation = controller_.getComputedTFOrientation();

    mavros_msgs::AttitudeTarget cur_att_target_;
    geometry_msgs::PoseStamped cur_pose_;
    cur_att_target_.orientation = output.orientation;
    cur_att_target_.thrust = output.force;
    cur_att_target_.header.stamp = ros::Time::now();
    cur_att_target_.header.frame_id = "map";
    cur_pose_.pose.position.x = feedback.cPosition.x();
    cur_pose_.pose.position.y = feedback.cPosition.y();
    cur_pose_.pose.position.z = feedback.cPosition.z();
    cur_pose_.pose.orientation = output.orientation;
    cur_pose_.header.frame_id = "map";
    cur_pose_.header.stamp = ros::Time::now();
    desi_att_pub_.publish(cur_pose_);
    att_pub_.publish(cur_att_target_);
}