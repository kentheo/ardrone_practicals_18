/*
 * Autopilot.cpp
 *
 *  Created on: 10 Jan 2017
 *      Author: sleutene
 */

#include <arp/Autopilot.hpp>
#include <arp/kinematics/operators.hpp>
#include <cmath>
#include <unistd.h>

namespace arp {

Autopilot::Autopilot(ros::NodeHandle& nh)
    : nh_(&nh)
{
  isAutomatic_ = false; // always start in manual mode

  // receive navdata
  subNavdata_ = nh.subscribe("ardrone/navdata", 50, &Autopilot::navdataCallback,
                             this);

  // commands
  pubReset_ = nh_->advertise<std_msgs::Empty>("/ardrone/reset", 1);
  pubTakeoff_ = nh_->advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
  pubLand_ = nh_->advertise<std_msgs::Empty>("/ardrone/land", 1);
  pubMove_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  //pubMove_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel_dummy", 1);
  pubPose_ = nh_->advertise<geometry_msgs::PoseStamped>("/ardrone/camera_pose",1);

  // flattrim service
  srvFlattrim_ = nh_->serviceClient<std_srvs::Empty>(
      nh_->resolveName("ardrone/flattrim"), 1);

  pose_stamped_seq = 0;
  PidController::Parameters PitchDefaultParams;
  PitchDefaultParams.k_p = 0.07;  // previous 0.07
  PitchDefaultParams.k_i = 0.02;  //0.08, 0.2
  PitchDefaultParams.k_d = 0.04; //0.04

  PidController::Parameters RollDefaultParams;
  RollDefaultParams.k_p = 0.07; // previous 0.07
  RollDefaultParams.k_i = 0.02;  //0.08
  RollDefaultParams.k_d = 0.04;

  PidController::Parameters VspeedDefaultParams;
  VspeedDefaultParams.k_p = 1.0;
  VspeedDefaultParams.k_i = 0.15;
  VspeedDefaultParams.k_d = 0.0;

  PidController::Parameters YawDefaultParams;
  YawDefaultParams.k_p = 1.5;
  YawDefaultParams.k_i = 0.0;
  YawDefaultParams.k_d = 0.0;


  PitchPID.setParameters(PitchDefaultParams);
  RollPID.setParameters(RollDefaultParams);
  VspeedPID.setParameters(VspeedDefaultParams);
  YawPID.setParameters(YawDefaultParams);
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
  if(navdata.altd < 600.0 && (navdata.state == arp::Autopilot::Flying ||
                              navdata.state == arp::Autopilot::Hovering ||
                              navdata.state == arp::Autopilot::Flying2)) {
    // so, effectively the ardrone_autonomy package is doing something weird. Let's fix.
    // \todo The 600 mm threshold is quite arbitrary -- this should not be hard-coded.
    navdata.state = arp::Autopilot::Unknown;
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
  //Horisontal position
  // point.x = r[2];
  // point.y = r[0];
  // point.z = r[1];
  //Vertical tag position
  point.x = r[0];
  point.y = r[1];
  point.z = r[2];

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

// Set to automatic control mode.
void Autopilot::setManual()
{
  isAutomatic_ = false;
}

// Set to manual control mode.
void Autopilot::setAutomatic()
{
  isAutomatic_ = true;
}

// Move the drone automatically.
bool Autopilot::setPoseReference(double x, double y, double z, double yaw)
{
  std::lock_guard<std::mutex> l(refMutex_);
  ref_x_ = x;
  ref_y_ = y;
  ref_z_ = z;
  ref_yaw_ = yaw;
  return true;
}

bool Autopilot::getPoseReference(double& x, double& y, double& z, double& yaw) {
  std::lock_guard<std::mutex> l(refMutex_);
  x = ref_x_;
  y = ref_y_;
  z = ref_z_;
  yaw = ref_yaw_;
  return true;
}

/// The callback from the estimator that sends control outputs to the drone
void Autopilot::controllerCallback(uint64_t timeMicroseconds,
                                  const arp::kinematics::RobotState& x)
{

  // get reference position
  Eigen::Vector3d ref_pos;//= { ref_pos_x, ref_pos_y, ref_pos_z};
  double ref_pos_yaw;

  //Print Current drone estimated position
  //std::cout<<"Current estimated Position: " << x.r_W[0]<<" "<< x.r_W[1]<<" "
              //  << x.r_W[2]<<" "<<kinematics::yawAngle(x.q_WS)<<std::endl;


  // Get waypoint list, if available
  //Bogdan: Not sure if we have to to anything here, for the above comment
  //{
  std::lock_guard<std::mutex> l(waypointMutex_);
  if(!waypoints_.empty()) {
    // TODO: setPoseReference() from current waypoint
    //B: Not sure if is a stack or a queue. If queue replace front() with back()
    Waypoint wp = waypoints_.front();
    //std::cout << "Waypoint coordinates :" << wp.x << " " << wp.y << " " << wp.z << " " << wp.yaw << std::endl;
    setPoseReference(wp.x, wp.y, wp.z, wp.yaw);

    getPoseReference(ref_pos[0], ref_pos[1], ref_pos[2], ref_pos_yaw);

    // TODO: remove the current waypoint, if the position error is below the tolerance.
    Eigen::Vector3d error = ref_pos - x.r_W;
    //std::cout << "Error is " << error[0] << std::endl;

    //B: Do we need some kind of modulus here? as we have to be in a certain circular range
    // from the desired position
    if(error.norm() < wp.posTolerance){
      //maybe pop_back()
      waypoints_.pop_front();
    }

  } else {
      // This is the original line of code:
      getPoseReference(ref_pos[0], ref_pos[1], ref_pos[2], ref_pos_yaw);
    }


  // only do anything here, if automatic
  if (!isAutomatic_) {
    // keep resetting this to make sure we use the current state as reference as soon as sent to automatic mode
    const double yaw = kinematics::yawAngle(x.q_WS);
    setPoseReference(x.r_W[0], x.r_W[1], x.r_W[2], yaw);
    return;
  }

  // Compute position error signal
  kinematics::Transformation T_WS(x.r_W, x.q_WS);   //use quaternion and/or inverse
  kinematics::Transformation T_SW = T_WS.inverse();
  Eigen::Matrix3d C_SW = T_SW.C();
  Eigen::Vector3d ref_x_WS= { ref_x_, ref_y_, ref_z_};
  Eigen::Vector3d e_r = C_SW * (ref_x_WS - x.r_W);

  // Compute yaw error
  double yaw = kinematics::yawAngle(x.q_WS);
  double e_yaw = ref_pos_yaw - yaw;
  // Wrap_around: Adjust yaw error to be within limit [-pi, pi]
  if (e_yaw < - M_PI) {
    e_yaw = e_yaw + (2 * M_PI);
  }
  else if (e_yaw > M_PI) {
    e_yaw = e_yaw - (2 * M_PI);
  }
  //std::cout << "e_r" << e_r[0] << e_r[1] << e_r[2]<< std::endl;
  // Compute error signal time derivatives
  Eigen::Vector3d e_dot = (-C_SW) * x.v_W;
  double e_dot_yaw = 0.0;


  // TODO: only enable when in flight
  DroneStatus status = droneStatus();
  //std::cout << "Drone status is " << status << std::endl;
  if (status == DroneStatus::Flying || status == DroneStatus::Hovering ||
              status == DroneStatus::Flying2) {
    // TODO: get ros parameter
    double max_phi, max_theta, max_velocity, max_rotation;

    //std::cout << "Phi, Theta, velocity, Rotation:" << max_phi << max_theta << max_velocity << max_rotation << std::endl;
    // Bool to check if the parameters reading are ok
    bool stop = false;
    if (! nh_->getParam("/ardrone_driver/euler_angle_max", max_phi)) { stop = true;}
    if (! nh_->getParam("/ardrone_driver/euler_angle_max", max_theta)) { stop = true;}
    if (! nh_->getParam("/ardrone_driver/control_vz_max", max_velocity)) { stop = true;}
    if (! nh_->getParam("/ardrone_driver/control_yaw", max_rotation)) { stop = true;}
    //std::cout << "The boolean stop is " << stop;
    if (!stop)
    {
      // convert max velocity from mm/s to m/s
      max_velocity /= 1000;

      //TODO: compute control output

      //Bogdan later edit: set max for PidControllers
      //Not sure what is the min
      //PHY = pitch =  X
      //Theta = roll = Y
      //State Output=[pitch,Roll,Vspeed, yaw]

      PitchPID.setOutputLimits(-max_phi, max_phi);
      RollPID.setOutputLimits(-max_theta, max_theta);
      YawPID.setOutputLimits(-max_rotation, max_rotation);
      VspeedPID.setOutputLimits(-max_velocity, max_velocity);
      // compute outputs from PID controller class members
      double output_Pitch, output_Roll, output_Yaw, output_Vspeed;
      output_Pitch = PitchPID.control(timeMicroseconds, e_r[0], e_dot[0]);  // e and e_dot to be provided
      output_Roll = RollPID.control(timeMicroseconds, e_r[1], e_dot[1]);  // e and e_dot to be provided
      output_Vspeed = VspeedPID.control(timeMicroseconds, e_r[2], e_dot[2]);  // e and e_dot to be provided
      output_Yaw = YawPID.control(timeMicroseconds, e_yaw, e_dot_yaw);  // e and e_dot to be provide

      //std::cout << "PRE: P, R, V, YAW:" << output_Pitch << output_Roll << output_Vspeed << output_Yaw << std::endl;

      // Scale these outputs by the ROS parameters obtained above
      output_Pitch /= max_phi;
      output_Roll /= max_theta;
      output_Vspeed /= max_velocity;
      output_Yaw /= max_rotation;

      // std::cout << "p, r, V, YAW:" << output_Pitch << " " << output_Roll <<
      //   " " << output_Vspeed << " " << output_Yaw << " " << std::endl;
      // TODO: send to move
       move(output_Pitch, output_Roll, output_Vspeed, output_Yaw);
       // move(1.0, 1.0, 1.0, 1.0);
    }
    else {
      std::cout << "Issue while reading ROS parameters, PID output not computed";
    }

  }








}

}  // namespace arp
