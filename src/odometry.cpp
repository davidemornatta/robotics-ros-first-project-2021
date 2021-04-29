#include "ros/ros.h"
#include "std_msgs/String.h"
#include "first_project/MotorSpeed.h"
#include "first_project/parametersConfig.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "tf/transform_broadcaster.h"
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <math.h>
#include <first_project/ResetPose.h>
#include <first_project/TeleportPose.h>

#define APPARENT_BASELINE 1.04
#define GEAR_REDUCTION 38
#define WHEEL_RADIUS 0.1575

typedef message_filters::sync_policies::ApproximateTime <first_project::MotorSpeed, first_project::MotorSpeed, first_project::MotorSpeed, first_project::MotorSpeed> MySyncPolicy;

typedef struct input_data {
    double vel_fl;
    double vel_fr;
    double vel_rl;
    double vel_rr;

    double time_fl;
    double time_fr;
    double time_rl;
    double time_rr;
} Input;

typedef struct pose {
    double x;
    double y;
    double theta;
} Pose;

typedef struct velocity {
    double speed_x;
    double speed_y;
    double omega;
} Velocity;


class scout_odometry {

private:
    tf::TransformBroadcaster odom_broadcaster;
    ros::NodeHandle pub_Node;
    ros::Publisher odometry_pub;
    ros::Publisher velocity_pub;
    ros::ServiceServer reset_service;
    ros::ServiceServer teleport_service;
    Pose currentPose;
    Velocity vel;
    double prev_vel;
    double prev_time;
    double deltaT;
    int approxType;
    std_msgs::String method;


public:
    scout_odometry(double x, double y, double theta) {
        currentPose.x = x;
        currentPose.y = y;
        currentPose.theta = theta;

        vel.speed_x = 0;
        vel.speed_y = 0;
        vel.omega = 0;

        prev_vel = 0;
        prev_time = 0;

        approxType = 0;
        method.data = "euler";

        odometry_pub = pub_Node.advertise<nav_msgs::Odometry>("/my_odom", 50);
        velocity_pub = pub_Node.advertise<geometry_msgs::TwistStamped>("/scout_velocity", 50);
        reset_service = pub_Node.advertiseService("reset_pose", &scout_odometry::reset, this);
        teleport_service = pub_Node.advertiseService("teleport_pose", &scout_odometry::teleport, this);
    }

    void compute_odometry(Input input) {
        if (approxType == 0) {
            method.data = "euler";
            compute_euler_odometry(input);
        } else if (approxType == 1) {
            method.data = "rk";
            compute_runge_kutta_odometry(input);
        }
    }

    void compute_euler_odometry(Input input) {
        double approxTime = (input.time_fl + input.time_fr + input.time_rl + input.time_rr) / 4;
        deltaT = approxTime - prev_time;
        prev_time = approxTime;

        double velLeft = -(input.vel_fl + input.vel_rl) / (2 * GEAR_REDUCTION);
        double velRight = (input.vel_fr + input.vel_rr) / (2 * GEAR_REDUCTION);
        double linearVel = (velLeft + velRight) / 2;

        currentPose.x += vel.speed_x * deltaT;
        currentPose.y += vel.speed_y * deltaT;
        currentPose.theta += vel.omega * deltaT;

        vel.speed_x = linearVel * cos(currentPose.theta);
        vel.speed_y = linearVel * sin(currentPose.theta);
        vel.omega = (velRight - velLeft) / APPARENT_BASELINE;
    }

    void compute_runge_kutta_odometry(Input input) {
        double approxTime = (input.time_fl + input.time_fr + input.time_rl + input.time_rr) / 4;
        deltaT = approxTime - prev_time;
        prev_time = approxTime;

        double velLeft = -(input.vel_fl + input.vel_rl) / (2 * GEAR_REDUCTION);
        double velRight = (input.vel_fr + input.vel_rr) / (2 * GEAR_REDUCTION);
        double linearVel = (velLeft + velRight) / 2;

        currentPose.x += prev_vel * deltaT * cos(currentPose.theta + vel.omega * deltaT / 2);
        currentPose.y += prev_vel * deltaT * sin(currentPose.theta + vel.omega * deltaT / 2);
        currentPose.theta += vel.omega * deltaT;

        vel.speed_x = linearVel * cos(currentPose.theta);
        vel.speed_y = linearVel * sin(currentPose.theta);
        vel.omega = (velRight - velLeft) / APPARENT_BASELINE;

        prev_vel = linearVel;
    }

    void publish_odometry() {
        nav_msgs::Odometry odom;
        ros::Time current_time = ros::Time::now();
        geometry_msgs::TransformStamped odom_trans;
        geometry_msgs::Quaternion odom_quaternion = tf::createQuaternionMsgFromYaw(currentPose.theta);

        odom.pose.pose.position.x = currentPose.x;
        odom.pose.pose.position.y = currentPose.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quaternion;

        odom.header.frame_id = "odom";
        odom.child_frame_id = "my_velocity";
        odom.twist.twist.linear.x = vel.speed_x;
        odom.twist.twist.linear.y = vel.speed_y;
        odom.twist.twist.angular.z = vel.omega;

        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = currentPose.x;
        odom_trans.transform.translation.y = currentPose.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quaternion;

        odom_broadcaster.sendTransform(odom_trans);
        odometry_pub.publish(odom);
    }

    void setApproxType(int chosenType) {
        if (chosenType == 0 || chosenType == 1) approxType = chosenType;
    }

    bool reset(first_project::ResetPose::Request &req, first_project::ResetPose::Response &res) {
        currentPose.x = 0.0;
        currentPose.y = 0.0;
        currentPose.theta = 0.0;

        vel.speed_x = 0;
        vel.speed_y = 0;
        vel.omega = 0;

        prev_vel = 0;
        prev_time = 0;
        return true;
    }

    bool teleport(first_project::TeleportPose::Request &req, first_project::TeleportPose::Response &res) {
        currentPose.x = req.x;
        currentPose.y = req.y;
        currentPose.theta = req.theta;

        vel.speed_x = 0;
        vel.speed_y = 0;
        vel.omega = 0;

        prev_vel = 0;
        prev_time = 0;
        return true;
    }
};

void param_callback(first_project::parametersConfig &config, uint32_t level, scout_odometry *scout) {
    scout->setApproxType(config.approx);
}

void
input_callback(const first_project::MotorSpeed::ConstPtr &m_fr, const first_project::MotorSpeed::ConstPtr &m_fl,
               const first_project::MotorSpeed::ConstPtr &m_rr,
               const first_project::MotorSpeed::ConstPtr &m_rl, scout_odometry *scout) {
    Input input;
    input.time_fl = m_fl->header.stamp.toSec();
    input.time_fr = m_fr->header.stamp.toSec();
    input.time_rl = m_rl->header.stamp.toSec();
    input.time_rr = m_rr->header.stamp.toSec();

    input.vel_fl = m_fl->rpm * 2 * M_PI * WHEEL_RADIUS / 60;
    input.vel_fr = m_fr->rpm * 2 * M_PI * WHEEL_RADIUS / 60;
    input.vel_rl = m_rl->rpm * 2 * M_PI * WHEEL_RADIUS / 60;
    input.vel_rr = m_rr->rpm * 2 * M_PI * WHEEL_RADIUS / 60;

    scout->compute_odometry(input);
    scout->publish_odometry();

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry");
    scout_odometry *scout = NULL;
    scout = new scout_odometry(0.0, 0.0, 0.0);

    dynamic_reconfigure::Server <first_project::parametersConfig> server;
    dynamic_reconfigure::Server<first_project::parametersConfig>::CallbackType f;
    f = boost::bind(&param_callback, _1, _2, scout);
    server.setCallback(f);

    ros::NodeHandle sub_Node;

    ros::ServiceClient reset_service_client = sub_Node.serviceClient<first_project::ResetPose>("reset_pose");
    ros::ServiceClient teleport_service_client = sub_Node.serviceClient<first_project::TeleportPose>("teleport_pose");

    message_filters::Subscriber <first_project::MotorSpeed> sub1(sub_Node, "/motor_speed_fr", 1);
    message_filters::Subscriber <first_project::MotorSpeed> sub2(sub_Node, "/motor_speed_fl", 1);
    message_filters::Subscriber <first_project::MotorSpeed> sub3(sub_Node, "/motor_speed_rr", 1);
    message_filters::Subscriber <first_project::MotorSpeed> sub4(sub_Node, "/motor_speed_rl", 1);

    message_filters::Synchronizer <MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2, sub3, sub4);
    sync.registerCallback(boost::bind(&input_callback, _1, _2, _3, _4, scout));

    ros::spin();
    return 0;
}
