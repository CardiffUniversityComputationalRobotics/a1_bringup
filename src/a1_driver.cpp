
// TODO:
// Need to visualize:
// > each leg position              \/
// > each foot sensor vector        \/
// > IMU data                       \/
// > body height                    \/
// > foot/forward speed             \both/
// > switch modes                   \/

// FIXME:
// > incorrect message stamps       \/
// > rewrite imu image filling through memcpy

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/WrenchStamped.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PolygonStamped.h" // represents leg positions
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "unitree_legged_msgs/FootVelocities.h"
#include "unitree_legged_msgs/FootForces.h"

using namespace UNITREE_LEGGED_SDK;

// FR FL RR RL
std::string footFrames[4] = {"FR_foot", "FL_foot", "RR_foot", "RL_foot"};
int time_stop = 0;

static geometry_msgs::Twist teleop_cmd;

void cmd_velCallback(const geometry_msgs::Twist msg)
{
    ROS_INFO("I heard: [%f]", msg.linear.x);
    time_stop = 0;
    teleop_cmd = msg;
}

class ROS_Publishers
{
public:
    ros::Publisher *a1_state;
    ros::Publisher *foot_force_pub;
    ros::Publisher *foot_velocity_pub;
    ros::Publisher *imu_pub;
    ros::Publisher *leg_pose;
    ros::Publisher *pose_pub;
    ros::Subscriber *cmd_vel_sub;
    int seq;
};

float lastForwVelocity = 0.0;
float lastSideVelocity = 0.0;

class Custom
{
public:
    Custom(uint8_t level) : safe(LeggedType::A1), udp(level)
    {
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl(ROS_Publishers rospub);

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    float dt = 0.002; // 0.001~0.01
};

void fillImuData(HighState &state, sensor_msgs::Imu &imuData, ROS_Publishers &rospub)
{
    // memcpy?
    imuData.linear_acceleration.x = state.imu.accelerometer[0];
    imuData.linear_acceleration.y = state.imu.accelerometer[1];
    imuData.linear_acceleration.z = state.imu.accelerometer[2];

    imuData.angular_velocity.z = state.rotateSpeed;

    if (lastForwVelocity == 0.0)
        lastForwVelocity = state.forwardSpeed;
    else if (lastSideVelocity == 0.0)
        lastSideVelocity = state.sideSpeed;
    else
    {
        imuData.linear_acceleration.y = (state.forwardSpeed - lastForwVelocity) / 0.02; // TODO: pass dt here
        imuData.linear_acceleration.x = (state.sideSpeed - lastSideVelocity) / 0.02;
    }

    if (state.imu.quaternion[0] == 0 && state.imu.quaternion[1] == 0 && state.imu.quaternion[2] == 0 && state.imu.quaternion[3] == 0)
    {
        imuData.orientation.w = 1;
        imuData.orientation.x = 0;
        imuData.orientation.y = 0;
        imuData.orientation.z = 0;
    }
    else
    {
        imuData.orientation.w = state.imu.quaternion[0];
        imuData.orientation.x = state.imu.quaternion[1];
        imuData.orientation.y = state.imu.quaternion[2];
        imuData.orientation.z = state.imu.quaternion[3];
    }
    imuData.header.seq = rospub.seq;
    imuData.header.frame_id = "imu_link";
    imuData.header.stamp = ros::Time::now();
}

void fillPolyData(HighState &state, geometry_msgs::PolygonStamped &legPolygon, ROS_Publishers &rospub)
{
    for (int leg = 0; leg < 4; leg++)
    {
        geometry_msgs::Point32 curLegPoint;
        std::memcpy(&curLegPoint, &state.footPosition2Body[leg], sizeof(Cartesian));
        legPolygon.polygon.points.push_back(curLegPoint);
    }
    legPolygon.header.frame_id = "trunk";
    legPolygon.header.seq = rospub.seq;
    legPolygon.header.stamp = ros::Time::now();
}

void SendToROS(Custom *a1Interface, ROS_Publishers rospub)
{
    HighState state = a1Interface->state;

    std_msgs::String a1_state_msg;              // a1_state
    unitree_legged_msgs::FootForces footForces; // foot forces
    unitree_legged_msgs::FootVelocities footVels;
    geometry_msgs::PolygonStamped legPolygon;
    geometry_msgs::PoseStamped a1_pose;
    sensor_msgs::Imu imuData;

    a1_state_msg.data = std::to_string(a1Interface->state.mode);

    //! FOOT FORCES AND VELOCITIES
    // FR FL RR RL
    // for (int leg = 0; leg < 4; leg++)
    // {
    //     // Constructing foot forces messages
    //     legForces[leg].wrench.force.z = state.footForce[leg]; // footForceEst ??
    //     legForces[leg].header.frame_id = footFrames[leg];
    //     legForces[leg].header.seq = rospub.seq;
    //     legForces[leg].header.stamp = ros::Time::now();
    //     // Constructing foot velocities messages
    //     legVels[leg].wrench.force.x = state.footSpeed2Body[leg].x; // not actually a force
    //     legVels[leg].wrench.force.y = state.footSpeed2Body[leg].y;
    //     legVels[leg].wrench.force.z = state.footSpeed2Body[leg].z;
    //     legVels[leg].header.frame_id = footFrames[leg];
    //     legVels[leg].header.seq = rospub.seq;
    //     legVels[leg].header.stamp = ros::Time::now();
    // }

    // ! foot forces messages
    footForces.fr_foot_force.force.z = state.footForce[0];
    footForces.fr_foot_force.header.seq = rospub.seq;
    footForces.fr_foot_force.header.stamp = ros::Time::now();
    footForces.fl_foot_force.force.z = state.footForce[1];
    footForces.rr_foot_force.force.z = state.footForce[2];
    footForces.rl_foot_force.force.z = state.footForce[3];

    // ! foot velocities

    //! fill imu data
    fillImuData(state, imuData, rospub);

    //! fill poly data
    fillPolyData(state, legPolygon, rospub);

    a1_pose.pose.position.x = state.forwardPosition;
    a1_pose.pose.position.y = state.sidePosition;
    a1_pose.pose.position.z = state.bodyHeight;

    a1_pose.pose.orientation.w = state.imu.quaternion[0];
    a1_pose.pose.orientation.x = state.imu.quaternion[1];
    a1_pose.pose.orientation.y = state.imu.quaternion[2];
    a1_pose.pose.orientation.z = state.imu.quaternion[3];

    a1_pose.header.frame_id = "base";
    a1_pose.header.seq = rospub.seq;
    a1_pose.header.stamp = ros::Time::now();

    // PUBLISHING
    for (int leg = 0; leg < 4; leg++)
    {
        rospub.foot_force_pub->publish(footForces);
        rospub.foot_velocity_pub->publish(footForces);
    }
    rospub.a1_state->publish(a1_state_msg);
    rospub.imu_pub->publish(imuData);
    rospub.leg_pose->publish(legPolygon);
    rospub.pose_pub->publish(a1_pose);

    rospub.seq++;
    ros::spinOnce();
}

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{
    udp.Send();
}

void Custom::RobotControl(ROS_Publishers rospub)
{
    if (!ros::ok())
        exit(1); // probably forbidden technique, but it works

    time_stop += 2;
    udp.GetRecv(state);

    cmd.forwardSpeed = 0.0f;
    cmd.sideSpeed = 0.0f;
    cmd.rotateSpeed = 0.0f;
    cmd.bodyHeight = 0.0f;

    // cmd.mode = 2;      // 0:idle, default stand      1:forced stand     2:walk continuously
    if (time_stop > 5000)
    {
        cmd.mode = 0;
    }
    else
    {
        cmd.mode = 2;
    }
    cmd.roll = 0;
    cmd.pitch = 0;
    cmd.yaw = 0;

    if ((teleop_cmd.linear.x >= 0) && (teleop_cmd.linear.x <= 1))
    {
        cmd.forwardSpeed = teleop_cmd.linear.x;
    }
    else if ((teleop_cmd.linear.x < 0) && (teleop_cmd.linear.x >= -0.7))
    {
        cmd.forwardSpeed = teleop_cmd.linear.x / 0.7f;
    }
    else
    {
        ROS_WARN("forward speed out of range: [%f]", teleop_cmd.linear.x);
    }

    if ((teleop_cmd.linear.y >= -0.4) && (teleop_cmd.linear.y <= 0.4))
    {
        cmd.sideSpeed = teleop_cmd.linear.y / 0.4f;
    }
    else
    {
        ROS_WARN("side speed out of range: [%f]", teleop_cmd.linear.y);
    }

    if ((teleop_cmd.angular.z >= -2.09) && (teleop_cmd.angular.z <= 2.09))
    {
        cmd.rotateSpeed = teleop_cmd.angular.z / 2.09f;
    }
    else
    {
        ROS_WARN("rotate speed out of range: [%f]", teleop_cmd.linear.y);
    }

    if (ros::ok())
        SendToROS(this, rospub);
    udp.SetSend(cmd);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "a1_driver");
    ros::NodeHandle nh;

    // !PUBLISHERS
    ros::Publisher a1_state_pub;
    //  Debug
    a1_state_pub = nh.advertise<std_msgs::String>("a1_state", 1000);
    // Forces
    ros::Publisher FRf_pub = nh.advertise<geometry_msgs::WrenchStamped>("FR_force", 1000);
    ros::Publisher FLf_pub = nh.advertise<geometry_msgs::WrenchStamped>("FL_force", 1000);
    ros::Publisher RRf_pub = nh.advertise<geometry_msgs::WrenchStamped>("RR_force", 1000);
    ros::Publisher RLf_pub = nh.advertise<geometry_msgs::WrenchStamped>("RL_force", 1000);
    // Velocities
    ros::Publisher FRv_pub = nh.advertise<geometry_msgs::WrenchStamped>("FR_vel", 1000);
    ros::Publisher FLv_pub = nh.advertise<geometry_msgs::WrenchStamped>("FL_vel", 1000);
    ros::Publisher RRv_pub = nh.advertise<geometry_msgs::WrenchStamped>("RR_vel", 1000);
    ros::Publisher RLv_pub = nh.advertise<geometry_msgs::WrenchStamped>("RL_vel", 1000);
    // IMU
    ros::Publisher IMU_pub = nh.advertise<sensor_msgs::Imu>("imu_raw", 1000);
    // Position 2 Body
    ros::Publisher LegPose_pub = nh.advertise<geometry_msgs::PolygonStamped>("feet_polygon", 1000);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1000);

    // !SUBSCRIBERS
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 1000, cmd_velCallback);

    /* ROS structure construction for loop */
    ROS_Publishers rospub; // a structure to pass into loop control
    rospub.a1_state = &a1_state_pub;
    rospub.leg_force_pub[0] = &FRf_pub; // packing all neccessary ros objects together
    rospub.leg_force_pub[1] = &FLf_pub;
    rospub.leg_force_pub[2] = &RRf_pub;
    rospub.leg_force_pub[3] = &RLf_pub;
    rospub.leg_velocity_pub[0] = &FRv_pub;
    rospub.leg_velocity_pub[1] = &FLv_pub;
    rospub.leg_velocity_pub[2] = &RRv_pub;
    rospub.leg_velocity_pub[3] = &RLv_pub;
    rospub.imu_pub = &IMU_pub;
    rospub.leg_pose = &LegPose_pub;
    rospub.pose_pub = &pose_pub;
    rospub.cmd_vel_sub = &cmd_vel_sub;
    rospub.seq = 0;

    Custom custom(HIGHLEVEL);

    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom, rospub));
    LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
    LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while (1)
    {
        sleep(1);
    };

    return 0;
}
