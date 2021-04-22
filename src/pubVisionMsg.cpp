#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "land_bridge/ControlCommand.h"

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

land_bridge::ControlCommand control_state;
void cmdState_cb(const land_bridge::ControlCommandConstPtr& msg){
    control_state = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::Subscriber vision_sub = nh.subscribe<land_bridge::ControlCommand>
            ("/LAA/mission/cmd", 10, cmdState_cb);

    // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //         ("mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/mavros/setpoint_velocity/cmd_vel", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::TwistStamped velocity;

    velocity.twist.linear.x = 1.0;
    velocity.twist.linear.z = 1.0;
    velocity.twist.angular.z = 0.2;

    // velocity.twist.linear.x = control_state.Reference_State.velocity_ref[0];
    // velocity.twist.linear.y = control_state.Reference_State.velocity_ref[1];
    // velocity.twist.linear.z = control_state.Reference_State.velocity_ref[2];
    // velocity.twist.angular.z = 0.2;


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_vel_pub.publish(velocity);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "STABILIZED";
    // set_mode_client.call(offb_set_mode);
    // std::cout<<"pos state"<< offb_set_mode.response.mode_sent << std::endl;
    
    // std::cout<<"POSITION CONTROL"<<std::endl;  
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(2.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("OFFBOARD enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(2.0))/**/){
                    //std::cout<<"current arm state: " << !current_state.armed <<std::endl;
                    //std::cout<<"Time: " << (ros::Time::now() - last_request > ros::Duration(5.0)) <<std::endl;
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_vel_pub.publish(velocity);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}