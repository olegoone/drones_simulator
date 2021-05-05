/*
 * Filename: DronesController.h
 *   Author: Oleg Maksimov
 *     Date: January 31, 2020
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef SRC_drones_controller_H_
#define SRC_drones_controller_H_


#include <functional>
#include <limits>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64.h>
#include <geometry_msgs/PoseStamped.h>


#include <airsim_ros_pkgs/Takeoff.h>
#include <airsim_ros_pkgs/Land.h>
#include <airsim_ros_pkgs/SetLocalPosition.h>
#include <airsim_ros_pkgs/SetGPSPosition.h>

#include <drones_controller/DroneWaypoint.h>
#include <drones_controller/DronePath.h>


namespace safety {


using namespace std;


/**
 * Safety controller
 */
class DronesController {

private:

	static const double MIN_TAKEOFF_ALTITUDE;

public:

	DronesController(ros::NodeHandle node);

	virtual ~DronesController();

public:

	// inline bool isLanded() const {
	// 	return mavrosExtendedState_.landed_state ==
	// 			mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
	// }

	// inline bool isMavlinkConnected() const {
	// 	return mavrosState_.connected;
	// }

	// inline bool isOffboardOn() const {
	// 	return mavrosState_.mode == MODE_OFFBOARD;
	// }

	bool takeoff(const double& altitude = MIN_TAKEOFF_ALTITUDE);

	bool land();

	bool sendLocalPositionGoal(const drones_controller::DroneWaypoint& waypoint);
	bool sendGlobalPositionGoal(const drones_controller::DroneWaypoint& waypoint);

	// bool sendLocalPositionGoal(const geometry_msgs::PoseStamped& goal);
	
	bool sendLocalPositionGoal(double x, double y, double z, double yaw);
	bool sendGlobalPositionGoal(double x, double y, double z, double yaw);

private:

	// void publishGoal(geometry_msgs::PoseStamped& goal);

	// void armCallback(const std_msgs::Empty);

	// void setModeCallback(const std_msgs::String);
	
	// void demoGotoGoalCallback(const std_msgs::Empty);

	// void setpointLocalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);

	void onWorldTick(const std_msgs::UInt64&);

	void takeoffCallback(const std_msgs::Empty);
	void landCallback(const std_msgs::Empty);

	void setDronePath(const drones_controller::DronePath& msg);
	
	// void setpointGlobalCallback(const sensor_msgs::NavSatFix& navSatMsg);

	// void stateCallback(const mavros_msgs::State::ConstPtr& state);

	// void localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose);

	// void homePositionCallback(const mavros_msgs::HomePosition& msg);


	/**
	 * Called by publish timer at constant rate 
	 */
	void publishGoalTimerCallback(const ros::TimerEvent&);

    // void checkOrientationLock(geometry_msgs::PoseStamped& goal);


	void onGoalReached(std_msgs::Empty);

private:


	ros::Subscriber onGoalReachedSubscriber_;

	ros::Subscriber takeoffSubscriber_;
	ros::Subscriber landSubscriber_;

	ros::Subscriber setPointLocalSubscriber_;

	/**
	 * Goal publisher timer
	 */
	ros::Timer publishTimer_;


	// vector<geometry_msgs::PoseStamped> waypoints_;
	vector<drones_controller::DroneWaypoint> waypoints_;

	double waypointWaitSeconds_ = 3.0;

	bool isPathCyclic_ = false;

	int cyclicPathLastWaypointExecutionWorldTick_ = 0;


	/**
	 * Is goal publishing enabled
	 */
	bool isEnabled_;

	/**
	 * The locked altitude the drone must preserve
	 */
	double altitudetLock_;

	/**
	 * Indicates that altitude is locked at the same
	 * value it was when offboard mode was enabled,
	 * otherwise, current altitude will be the limit
	 */
	bool altitudeLockEnabled_;

	/**
	 * Indicates that heading value is locked
	 * in goal messages
	 */
	bool headingLockEnabled_;

	/**
	 * Locked heading angle
	 */
	double headingLock_;

	/**
	 * Is connected to ground station
	 */
	// bool gcsConnection_;

	/**
	 * Flight controller state
	 */
	// mavros_msgs::State mavrosState_;

	/**
	 * Position of the drone
	 */
	geometry_msgs::PoseStamped localPose_;

	/**
	 * Flight controller extended state (for landed_state)
	 */
	// mavros_msgs::ExtendedState mavrosExtendedState_;

	/**
	 * Arming service client
	 */
	ros::ServiceClient setArmingStateService_;

	/**
	 * Takeoff service client
	 */
	ros::ServiceClient takeoffService_;

	/**
	 * SetMode service client
	 */
	ros::ServiceClient setModeService_;

	/**
	 * Land service client
	 */
	ros::ServiceClient landService_;

	ros::ServiceClient localPositionGoalService_;

	ros::ServiceClient globalPositionGoalService_;

	/**
	 * Goal publisher
	 */
	ros::Publisher setpointPublisher_;

	ros::Publisher onGoalReachedPublisher_;

	ros::Publisher onWaypointReachedPublisher_;


	ros::Subscriber onWorldTickSubscriber_;

	
	bool waypointReached_ = true;

	unsigned int worldTick_ = 0;
};


} /* namespace safety */


#endif /* SRC_drones_controller_H_ */
