/*
 * Filename: DronesController.cpp
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

#include <drones_controller/DronesController.h>


namespace safety {

const double DronesController::MIN_TAKEOFF_ALTITUDE = 2.5;

DronesController::DronesController(ros::NodeHandle node): 
    waypoints_(vector<drones_controller::DroneWaypoint>()),
    altitudetLock_(0),
    altitudeLockEnabled_(false), headingLock_(0), headingLockEnabled_(false) {

	double publishRate;

	ros::NodeHandle nodePrivate("~");

    // nodePrivate.param<std::string> ("local_frame", localFrame_, "local");

	node.param("publish_rate", publishRate, 10.0);
    // node.param("goal_tolerance", goalTolerance_, 1.0);
    // node.param("min_height", minAllowedHeight_, 2.5);
    // node.param("max_height", maxAllowedHeight_, 50.0);
    // node.param("orientation_lock_distance", orientationLockDistance_, 2.0);


    takeoffService_ = node.serviceClient<airsim_ros_pkgs::Takeoff>("takeoff", false);
    landService_ = node.serviceClient<airsim_ros_pkgs::Land>("land", false);

    localPositionGoalService_ = node.serviceClient<airsim_ros_pkgs::SetLocalPosition>("local_position_goal/override", false);
    globalPositionGoalService_ = node.serviceClient<airsim_ros_pkgs::SetGPSPosition>("gps_goal/override", false);

	onGoalReachedSubscriber_ = node.subscribe("goal_reached", 1, &DronesController::onGoalReached, this);

	onWorldTickSubscriber_ = node.subscribe("/world_tick", 1, &DronesController::onWorldTick, this);

    takeoffSubscriber_ = node.subscribe("takeoff", 1, &DronesController::takeoffCallback, this);
    landSubscriber_ = node.subscribe("takeoff", 1, &DronesController::landCallback, this);

    setPointLocalSubscriber_ = node.subscribe("set_path", 1, &DronesController::setDronePath, this);

	// gcsConnectionSubscriber_ = node.subscribe("gcs/connection", 1, &DronesController::gcsConnectionCallback, this);
	// gcsTimeSubscriber_ = node.subscribe("/gcs/time", 1, &DronesController::gcsTimeCallback, this);

	publishTimer_ = node.createTimer(ros::Duration(1.0 / publishRate), &DronesController::publishGoalTimerCallback, this);

    sleep(1);
    
    takeoff();

    // sendLocalPositionGoal(20, 1100, -100, 0.0);

    // sleep(30);

    // land();
}

DronesController::~DronesController() {

}


bool DronesController::takeoff(const double& altitude /* = MIN_TAKEOFF_ALTITUDE*/) {

    airsim_ros_pkgs::Takeoff takeoffSrvMsg;
    
    takeoffSrvMsg.request.waitOnLastTask = false;

    if (!takeoffService_.call(takeoffSrvMsg) || !takeoffSrvMsg.response.success) {
        // ROS_ERROR("Safety: takeoff service call failed");

        // return false;
    }

    return true;
}

void DronesController::setDronePath(const drones_controller::DronePath& msg) {

    waypoints_ = msg.waypoints;
    std::reverse(waypoints_.begin(), waypoints_.end());

    if (waypoints_.size() == 0) {

        cerr << "Received empty waypoints list" << endl;

        return;
    }

    //msg.isPathCyclic
    // cerr << "isPathCyclic: " << msg.isPathCyclic << endl;

    isPathCyclic_ = msg.isPathCyclic;

    if (isPathCyclic_) {
        cerr << " Path is cyclic " << endl;
    }
    else {
        cerr << " Path is not cyclic " << endl;
    }

    // cyclicPathLastWaypointExecutionTick_ = waypoints_.front().minExecutionTick;

    cerr << "waypoints_ size = " << waypoints_.size() << endl;

    waypointWaitSeconds_ = msg.waypointWaitSeconds;

    sendLocalPositionGoal(waypoints_.back());
    // sendGlobalPositionGoal(waypoints_.back());

    // Immediate start
    // lastGoalArrivalTime_ = ros::Time::now().toSec() - waypointWaitSeconds_ - 1;

	// goalReceived_ = true;

    // arm();

	// setMode("OFFBOARD");
}

bool DronesController::land() {

    airsim_ros_pkgs::Land landSrvMsg;
    
    landSrvMsg.request.waitOnLastTask = false;

    if (!landService_.call(landSrvMsg) || !landSrvMsg.response.success) {
        // ROS_ERROR("Safety: land service call failed");

        // return false;
    }

    return true;
}

bool DronesController::sendLocalPositionGoal(const drones_controller::DroneWaypoint& waypoint) {

    return sendLocalPositionGoal(
            waypoint.goalPose.pose.position.x,
            waypoint.goalPose.pose.position.y,
            waypoint.goalPose.pose.position.z,
            waypoint.goalPose.pose.orientation.z);
}

bool DronesController::sendGlobalPositionGoal(const drones_controller::DroneWaypoint& waypoint) {

    return sendGlobalPositionGoal(
            waypoint.goalPose.pose.position.x,
            waypoint.goalPose.pose.position.y,
            waypoint.goalPose.pose.position.z,
            waypoint.goalPose.pose.orientation.z);
}

bool DronesController::sendLocalPositionGoal(double x, double y, double z, double yaw) {

    airsim_ros_pkgs::SetLocalPosition localPositionGoalMsg;

    localPositionGoalMsg.request.x = x;
    localPositionGoalMsg.request.y = y;
    localPositionGoalMsg.request.z = z;
    localPositionGoalMsg.request.yaw = yaw;
    localPositionGoalMsg.request.vehicle_name = ""; //TODO:

    waypointReached_ = false;

    if (!localPositionGoalService_.call(localPositionGoalMsg) || !localPositionGoalMsg.response.success) {

    }

    cyclicPathLastWaypointExecutionWorldTick_ = worldTick_;

    return true;
}

bool DronesController::sendGlobalPositionGoal(double x, double y, double z, double yaw) {

    airsim_ros_pkgs::SetGPSPosition globalPositionGoalMsg;

    globalPositionGoalMsg.request.latitude = x;
    globalPositionGoalMsg.request.longitude = y;
    globalPositionGoalMsg.request.altitude = z;
    globalPositionGoalMsg.request.yaw = yaw;
    globalPositionGoalMsg.request.vehicle_name = ""; //TODO:

    waypointReached_ = false;

    if (!globalPositionGoalService_.call(globalPositionGoalMsg) || !globalPositionGoalMsg.response.success) {

    }

    cyclicPathLastWaypointExecutionWorldTick_ = worldTick_;

    return true;
}

// void DronesController::publishGoal(geometry_msgs::PoseStamped& goal) {
    
// }

void DronesController::takeoffCallback(std_msgs::Empty msg) {
    takeoff();
}
void DronesController::landCallback(std_msgs::Empty msg) {
    land();
}

void DronesController::onWorldTick(const std_msgs::UInt64& msg) {

    worldTick_ = msg.data;
}

void DronesController::onGoalReached(std_msgs::Empty msg) {

    // waypointWaitSeconds_

    if (!isPathCyclic_) {

        waypoints_.pop_back();
    }
    else {

        std::rotate(waypoints_.begin(), waypoints_.begin()+1, waypoints_.end());
    }

    if (waypoints_.empty()) {
        // onGoalReached();
        cerr << " all waypoints reached " << endl;
    }
    else {
        cerr << "waypoints_ size: " << waypoints_.size() << endl;
    }
    
    waypointReached_ = true;
}

void DronesController::publishGoalTimerCallback(const ros::TimerEvent&) {

    if (!waypoints_.empty()) {

        cerr << "Current waipoint tick: " << waypoints_.back().minExecutionTick << "; World tick: " << worldTick_ << endl;
    }

    if (!waypointReached_ || waypoints_.empty()) {
        return;
    }


cerr << "worldTick_: " << (worldTick_) << endl;
cerr << "(cyclicPathLastWaypointExecutionWorldTick_ + waypoints_.back().minExecutionTick)): " << (cyclicPathLastWaypointExecutionWorldTick_ + waypoints_.back().minExecutionTick) << endl;
cerr << "cyclicPathLastWaypointExecutionWorldTick_: " << (cyclicPathLastWaypointExecutionWorldTick_) << endl;

    if ((!isPathCyclic_ && worldTick_ < waypoints_.back().minExecutionTick) ||
        (isPathCyclic_  && worldTick_ < (cyclicPathLastWaypointExecutionWorldTick_ + waypoints_.back().minExecutionTick))) {

        cerr << "Waiting for tick: " << waypoints_.back().minExecutionTick << endl;
        return;
    }

    cerr << " sending next goal " << endl;

    sendLocalPositionGoal(waypoints_.back());
    // sendGlobalPositionGoal(waypoints_.back());
}

} /* namespace safety */

