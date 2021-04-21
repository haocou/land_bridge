#include <ros/ros.h>

#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <land_bridge/DroneState.h>

#include <tf/transform_datatypes.h>

// for simutaneously subscribe the topic
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

land_bridge::DroneState uavAllState;
std_msgs::Float64 uavRelAlt;
ros::Publisher uavStatePub;
void uavStateCallBack(const mavros_msgs::StateConstPtr& uavState,
					  const geometry_msgs::PoseStampedConstPtr& uavPose,
					  const geometry_msgs::TwistStampedConstPtr& uavVel);

void uavAltCallback(const std_msgs::Float64ConstPtr& uavRelAlt);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "droneState");

	ros::NodeHandle nh;

	uavStatePub = nh.advertise<land_bridge::DroneState>("/LAA/drone_state", 100);
	ros::Subscriber uavRelAlt_sub_ = nh.subscribe<std_msgs::Float64>("/mavros/global_position/rel_alt", 10, uavAltCallback);

	typedef message_filters::sync_policies::ApproximateTime<mavros_msgs::State, geometry_msgs::PoseStamped, 
	geometry_msgs::TwistStamped> landBridgeSyncPolicy;
	message_filters::Subscriber<mavros_msgs::State>* uavState_sub_ ;
	message_filters::Subscriber<geometry_msgs::PoseStamped>* uavPose_sub_;
	message_filters::Subscriber<geometry_msgs::TwistStamped>* uavVel_sub_ ;
	message_filters::Synchronizer<landBridgeSyncPolicy>* sync_;

	uavState_sub_ = new message_filters::Subscriber<mavros_msgs::State>(nh, "/mavros/state", 10);
    uavPose_sub_  = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh, "/mavros/local_position/pose", 10);
	uavVel_sub_ = new message_filters::Subscriber<geometry_msgs::TwistStamped>(nh, "/mavros/local_position/velocity_local", 10);
   
    sync_ = new  message_filters::Synchronizer<landBridgeSyncPolicy>
								(landBridgeSyncPolicy(10), *uavState_sub_, *uavPose_sub_, *uavVel_sub_);
	sync_->registerCallback(boost::bind(&uavStateCallBack, _1, _2, _3));

	ros::spin();

	return 0;
}


void uavStateCallBack(const mavros_msgs::StateConstPtr& uavState,
					  const geometry_msgs::PoseStampedConstPtr& uavPose,
					  const geometry_msgs::TwistStampedConstPtr& uavVel)
{
	uavAllState.header.frame_id = uavState->header.frame_id;
	uavAllState.header.seq = uavState->header.seq;
	uavAllState.header.stamp = uavState->header.stamp;

	uavAllState.time_from_start = int64_t((ros::Time::now().toSec()) );

	uavAllState.connected = uavState->connected;
	uavAllState.armed = uavState->armed;
	uavAllState.mode = uavState->mode;
	uavAllState.landed = false;
	if (uavState->mode == "AUTO.LAND")
	{
		uavAllState.landed = true;
	}

	uavAllState.position[0] = uavPose->pose.position.x;
	uavAllState.position[1] = uavPose->pose.position.y;
	uavAllState.position[2] = uavPose->pose.position.z;

	uavAllState.attitude_q.w = uavPose->pose.orientation.w;
	uavAllState.attitude_q.x = uavPose->pose.orientation.x;
	uavAllState.attitude_q.y = uavPose->pose.orientation.y;
	uavAllState.attitude_q.z = uavPose->pose.orientation.z;

	uavAllState.velocity[0] = uavVel->twist.linear.x;
	uavAllState.velocity[1] = uavVel->twist.linear.y;
	uavAllState.velocity[2] = uavVel->twist.linear.z;

	uavAllState.attitude_rate[0] = uavVel->twist.angular.x;
	uavAllState.attitude_rate[1] = uavVel->twist.angular.y;
	uavAllState.attitude_rate[2] = uavVel->twist.angular.z;

	uavAllState.rel_alt = uavRelAlt.data;

	//transform quaterion to euler angle
	tf::Quaternion quat;
    tf::quaternionMsgToTF(uavPose->pose.orientation, quat);

	double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	uavAllState.attitude[0] = roll;
	uavAllState.attitude[1] = pitch;
	uavAllState.attitude[2] = yaw;

	uavStatePub.publish(uavAllState);
}


void uavAltCallback(const std_msgs::Float64ConstPtr& RelAlt)
{
	uavRelAlt.data = RelAlt->data;
}