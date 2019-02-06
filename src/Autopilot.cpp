/*
 * Autopilot.cpp
 *
 *  Created on: 10 Jan 2017
 *      Author: sleutene
 */

#include <arp/Autopilot.hpp>

namespace arp {

Autopilot::Autopilot(ros::NodeHandle& nh)
    : nh_(&nh)
{
  // receive navdata
  subNavdata_ = nh.subscribe("ardrone/navdata", 50, &Autopilot::navdataCallback,
                             this);

  // commands
  pubReset_ = nh_->advertise<std_msgs::Empty>("/ardrone/reset", 1);
  pubTakeoff_ = nh_->advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
  pubLand_ = nh_->advertise<std_msgs::Empty>("/ardrone/land", 1);
  pubMove_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  pubPose_ = nh_->advertise<geometry_msgs::PoseStamped>("/ardrone/camera_pose",1);

  // flattrim service
  srvFlattrim_ = nh_->serviceClient<std_srvs::Empty>(
      nh_->resolveName("ardrone/flattrim"), 1);

  pose_stamped_seq = 0;

}

void Autopilot::navdataCallback(const ardrone_autonomy::NavdataConstPtr& msg)
{
  std::lock_guard<std::mutex> l(navdataMutex_);
  lastNavdata_ = *msg;
}

// Get the drone status.
Autopilot::DroneStatus Autopilot::droneStatus()
{
  ardrone_autonomy::Navdata navdata;
  {
    std::lock_guard<std::mutex> l(navdataMutex_);
    navdata = lastNavdata_;
  }
  return DroneStatus(navdata.state);
}

// Get the remaining battery percentage
float Autopilot::batteryStatus()
{
  ardrone_autonomy::Navdata navdata;
  {
    std::lock_guard<std::mutex> l(navdataMutex_);
    navdata = lastNavdata_;
  }
  return navdata.batteryPercent;
}

// Request flattrim calibration.
bool Autopilot::flattrimCalibrate()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  // ARdrone -> flattrim calibrate
  std_srvs::Empty flattrimServiceRequest;
  srvFlattrim_.call(flattrimServiceRequest);
  return true;
}

// Takeoff.
bool Autopilot::takeoff()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  // ARdrone -> take off
  std_msgs::Empty takeoffMsg;
  pubTakeoff_.publish(takeoffMsg);
  return true;
}

// Land.
bool Autopilot::land()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed && status != DroneStatus::Landing
      && status != DroneStatus::Looping) {
    // ARdrone -> land
    std_msgs::Empty landMsg;
    pubLand_.publish(landMsg);
    return true;
  }
  return false;
}

// Turn off all motors and reboot.
bool Autopilot::estopReset()
{
  // ARdrone -> Emergency mode
  std_msgs::Empty resetMsg;
  pubReset_.publish(resetMsg);
  return true;
}

// Move the drone manually.
bool Autopilot::manualMove(double forward, double left, double up,
                           double rotateLeft)
{
  return move(forward, left, up, rotateLeft);
}

// returns value clamped to interval
float clamp(double lower, double value, double upper)
{
  if (value > upper)
    return upper;
  else if (value < lower)
    return lower;
  else
    return value;
}

bool Autopilot::publishTag(arp::Frontend::Detection det){
  geometry_msgs::PoseStamped pose_stamped;
  std_msgs::Header header;
  geometry_msgs::Pose pose;

  kinematics::Transformation T_TC = det.T_CT.inverse();
  header.seq = pose_stamped_seq;
  pose_stamped_seq = pose_stamped_seq + 1;


  header.frame_id = "target";

  //header.stamp = ros::Time::now(); // check this
  header.stamp.sec = (int) time(NULL);
  pose_stamped.header = header;

  Eigen::Vector3d r =  T_TC.r();

  geometry_msgs::Point point;
  point.x = r[2];
  point.y = r[0];
  point.z = r[1];

  Eigen::Quaterniond q =  T_TC.q();
  geometry_msgs::Quaternion quat;

  quat.x = q.x();
  quat.y = q.y();
  quat.z = q.z();
  quat.w = q.w();

  pose.position = point;
  pose.orientation = quat;
  pose_stamped.pose = pose;

  pubPose_.publish(pose_stamped);

  std::cout << "pose published" << std::endl;
  return true;
}
// Move the drone.
bool Autopilot::move(double forward, double left , double up, double rotateLeft)
{
    DroneStatus status = droneStatus();
    if (status == DroneStatus::Flying || status==DroneStatus::Flying2
        || status == DroneStatus::Hovering){
            geometry_msgs::Twist moveMsg;
            moveMsg.linear.x = forward;
            moveMsg.linear.y = left;
            moveMsg.linear.z = up;
            moveMsg.angular.z = rotateLeft;
            pubMove_.publish(moveMsg);
            return true;
        }
  return false;
}

}  // namespace arp
