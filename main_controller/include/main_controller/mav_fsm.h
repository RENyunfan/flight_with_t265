//
// Created by yunfan on 2020/12/22.
//

#ifndef MAIN_CONTROLLER_MAV_FSM_H
#define MAIN_CONTROLLER_MAV_FSM_H
#include "main_controller/common_include.h"
#include "main_controller/SO3Control.h"
#include "main_controller/calculate.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "geometry_msgs/Quaternion.h"
namespace wtr{

    class MavFsmNode{
    private:
        int cnt;
        ros::NodeHandle node_;
        ros::Subscriber vision_sub_,rc_sub_,odom_sub_,imu_sub_;
        ros::Publisher local_pos_pub_,speed_pub_, att_pub_,desi_att_pub_;
        ros::Timer mav_fsm_;
        geometry_msgs::Pose init_pose_;
        geometry_msgs::PoseStamped takeoff_pose_;
        geometry_msgs::TwistStamped land_signal_;
        bool is_init_pose_;

        SO3Control controller_;
        cal_c cal_;

        struct desired_t{
            Vec3 cPosition;
            Vec3 cPositionD;
            Vec3 cPositionDD;

            Mat33 rotation;
            Vec3 bodyRate;
            double thrust;
            double yaw;
            double yawD;
        }desired;


        struct feedback_t{
            Vec3 cPosition;
            Vec3 cPositionD;
            Vec3 cPositionDD;

            Mat33 rotation;
            Vec3 bodyRate;
            double thrust;
            double yaw;
            double yawD;
        }feedback;

        struct output_t{
            double force;
            Eigen::Quaterniond quad;
            geometry_msgs::Quaternion orientation;
        }output;

        struct fly_params_t{
            Vec3 k_position,k_velocity;
        }fp;

        ros::Time now_;


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


        void rcCallback(const mavros_msgs::RCInConstPtr &msg);
        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void imuCallback(const  sensor_msgs::ImuConstPtr & imu_msg);
        void odomCallback(const nav_msgs::OdometryConstPtr &  odom_msg);
        void main_FSM(const ros::TimerEvent& /*event*/);
        void updateOutputOrientation();
        void callSO3ControlOnce();
        /*  Define the mode callback functions  */
        void att_mode();
        void land_mode();
        void takeoff_mode();
        void wait_mode();

    public:
        MavFsmNode(ros::NodeHandle & nh);
        ~MavFsmNode(){};
        typedef std::shared_ptr<MavFsmNode> Ptr;

    };

}



#endif //MAIN_CONTROLLER_MAV_FSM_H
