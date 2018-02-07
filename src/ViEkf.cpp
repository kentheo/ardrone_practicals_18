/*
 * ViEkf.cpp
 *
 *  Created on: 20 Nov 2015
 *      Author: sleutene
 */

#include <opencv2/highgui/highgui.hpp>

#include <arp/ViEkf.hpp>
#include <arp/kinematics/Imu.hpp>

namespace arp {

ViEkf::ViEkf()
    : cameraModel_(0, 0, 0, 0, 0, 0, arp::cameras::NoDistortion())
{
  x_.setZero();
  P_.setZero();
}

// Add a marker target.
bool ViEkf::setTarget(unsigned int j,
                      const arp::kinematics::Transformation & T_WTj,
                      double targetSizeMetres)
{
  T_WT_[j] = T_WTj;
  targetSizesMetres_[j] = targetSizeMetres;
  return true;
}

// Remove a marker target.
bool ViEkf::removeTarget(unsigned int j)
{
  return 2 == T_WT_.erase(j) + targetSizesMetres_.erase(j);  // both have to erase one element...
}

// Set the pose of the camera relative to the IMU.
bool ViEkf::setCameraExtrinsics(const arp::kinematics::Transformation & T_SC)
{
  T_SC_ = T_SC;
  return true;
}

// Set the intrinsics of the camera, i.e. the camera model
bool ViEkf::setCameraIntrinsics(
    const arp::cameras::PinholeCamera<arp::cameras::NoDistortion> & cameraModel)
{
  cameraModel_ = cameraModel;
  return true;
}

// Set IMU noise parameters
void ViEkf::setImuNoiseParameters(double sigma_c_gyr, double sigma_c_acc,
                                  double sigma_c_gw, double sigma_c_aw)
{
  sigma_c_gyr_ = sigma_c_gyr;
  sigma_c_acc_ = sigma_c_acc;
  sigma_c_gw_ = sigma_c_gw;
  sigma_c_aw_ = sigma_c_aw;
}

// Set measurement noise parameter.
void ViEkf::setDetectorNoiseParameter(double sigma_imagePoint)
{
  sigma_imagePoint_ = sigma_imagePoint;
}

// Initialise the states
bool ViEkf::initialiseState(uint64_t timestampMicroseconds,
                            const arp::kinematics::RobotState & x,
                            const Eigen::Matrix<double, 15, 15> & P)
{
  timestampLastUpdateMicrosec_ = timestampMicroseconds;
  x_ = x;
  P_ = P;

  // reset propagation for publishing
  x_propagated_ = x_;
  timestampPropagatedMicrosec_ = timestampLastUpdateMicrosec_;

  return true;
}

// Has this EKF been initialised?
bool ViEkf::isInitialised() const
{
  return 0 != timestampLastUpdateMicrosec_;
}

// Get the states.
bool ViEkf::getState(uint64_t timestampMicroseconds,
                     arp::kinematics::RobotState & x,
                     Eigen::Matrix<double, 15, 15> * P)
{
  if (timestampPropagatedMicrosec_ == 0) {
    x = x_;
    if (P) {
      *P = P_;
    }
    return false;
  }

  if (timestampPropagatedMicrosec_ > timestampLastUpdateMicrosec_) {
    if (timestampPropagatedMicrosec_ - timestampLastUpdateMicrosec_ > 100000) {
      // stop propagation, this will just diverge
      // assign output
      x = x_propagated_;
      if (P) {
        *P = P_;  // Not 100% correct, we should have also propagated  P_...
      }
      return false;
    }
  }

  // run prediction as far as possible
  for (auto it_k_minus_1 = imuMeasurementDeque_.begin();
      it_k_minus_1 != imuMeasurementDeque_.end(); ++it_k_minus_1) {
    auto it_k = it_k_minus_1;
    it_k++;
    if (it_k == imuMeasurementDeque_.end()) {
      return false;  // we reached the buffer end...
    }

    // ensure we're in the right segment
    if (it_k->timestampMicroseconds < timestampPropagatedMicrosec_) {
      continue;
    }

    if (it_k->timestampMicroseconds >= timestampMicroseconds) {
      break;
    }

    // propagate and get state transition matrix
    arp::kinematics::ImuKinematicsJacobian F =
        arp::kinematics::ImuKinematicsJacobian::Identity();
    arp::kinematics::RobotState x_start = x_propagated_;
    arp::kinematics::Imu::stateTransition(x_start, *it_k_minus_1, *it_k,
                                          x_propagated_, &F);
  }
  // assign output
  x = x_propagated_;
  if (P) {
    *P = P_;  // Not 100% correct, we should have also propagated  P_...
  }

  // remember
  timestampPropagatedMicrosec_ = timestampMicroseconds;
  return true;
}

// Obtain the target pose of target id.
bool ViEkf::get_T_WT(int id, kinematics::Transformation & T_WT) const
{
  // check if target was registered:
  if (targetSizesMetres_.find(id) == targetSizesMetres_.end()) {
    return false;
  }
  T_WT = T_WT_.at(id);
  return true;
}

bool ViEkf::addImuMeasurement(uint64_t timestampMicroseconds,
                              const Eigen::Vector3d & gyroscopeMeasurements,
                              const Eigen::Vector3d & accelerometerMeasurements)
{
  imuMeasurementDeque_.push_back(
      arp::kinematics::ImuMeasurement { timestampMicroseconds,
          gyroscopeMeasurements, accelerometerMeasurements });
  return true;
}

// The EKF prediction.
bool ViEkf::predict(uint64_t from_timestampMicroseconds,
                    uint64_t to_timestampMicroseconds)
{
  for (auto it_k_minus_1 = imuMeasurementDeque_.begin();
      it_k_minus_1 != imuMeasurementDeque_.end(); ++it_k_minus_1) {
    auto it_k = it_k_minus_1;
    it_k++;
    if (it_k == imuMeasurementDeque_.end()) {
      break;  // we reached the buffer end...
    }

    // ensure we're in the right segment
    if (it_k->timestampMicroseconds < from_timestampMicroseconds
        || it_k->timestampMicroseconds >= to_timestampMicroseconds) {
      continue;
    }

    // access the IMU measurements:
    kinematics::ImuMeasurement z_k_minus_1 = *it_k_minus_1; // IMU meas. at last time step
    kinematics::ImuMeasurement z_k = *it_k; // IMU measurements at current time step

    // get the time delta
    const double delta_t = double(z_k.timestampMicroseconds - z_k_minus_1.timestampMicroseconds) * 1.0e-6;

    // TODO: propagate robot state x_ using IMU measurements
    // i.e. we do x_k = f(x_k_minus_1).
    // Also, we compute the matrix F (linearisation of f()) related to
    // delta_chi_k = F * delta_chi_k_minus_1.

    // TODO: propagate covariance matrix P_

  }
  return false;  // TODO: change to true once implemented
}

// Pass a set of corner measurements to trigger an update.
bool ViEkf::addTargetMeasurement(
    uint64_t timestampMicroseconds,
    const Eigen::Matrix<double, 2, 4> & imagePointMeasurements, int id)
{
  // let's do the propagation from last time to now:
  predict(timestampLastUpdateMicrosec_, timestampMicroseconds);

  // check if target was registered:
  if (targetSizesMetres_.find(id) == targetSizesMetres_.end()) {
    return false;
  }

  // now we are ready to do the actual update
  bool success = true;
  for (size_t k = 0; k < 4; ++k) {
    success &= update(timestampMicroseconds, imagePointMeasurements, id, k);
  }

  // monitor update history
  if (!success) {
    lastTimeReject_ = true;
    if (lastTimeReject_) {
      rejections_++;
    }
  } else {
    // reset monitoring
    lastTimeReject_ = false;
    rejections_ = 0;
  }

  // remember update
  timestampLastUpdateMicrosec_ = timestampMicroseconds;
  auto it = imuMeasurementDeque_.begin();

  // also delete stuff in the queue since not needed anymore.
  while (it != imuMeasurementDeque_.end()) {
    auto it_p1 = it;
    it_p1++;
    if (it_p1 == imuMeasurementDeque_.end()) {
      break;
    }
    if (it->timestampMicroseconds < timestampMicroseconds
        && it_p1->timestampMicroseconds >= timestampMicroseconds) {
      break;
    }
    it++;
  }
  imuMeasurementDeque_.erase(imuMeasurementDeque_.begin(), it);

  // reset propagation for publishing
  x_propagated_ = x_;
  timestampPropagatedMicrosec_ = timestampLastUpdateMicrosec_;

  if (rejections_ > 3) {
    std::cout << "REINITIALISE" << std::endl;
    timestampLastUpdateMicrosec_ = 0;  // not initialised.
    rejections_ = 0;
    lastTimeReject_ = false;
  }
  return true;
}

// The EKF update.
bool ViEkf::update(uint64_t timestampMicroseconds,
                   const Eigen::Matrix<double, 2, 4> & imagePointMeasurements,
                   int id, unsigned int p)
{

  // imagePointMeasurements are the corners of the tag we found in the
  // image (undistorted, in pixels [u,v]). We are requested to do an
  // update using the p-th corner (i.e. one at the time).

  // pose T_WS: transforms points represented in Sensor(IMU) coordinates
  // into World coordinates
  kinematics::Transformation T_WS(x_.r_WS_W, x_.q_WS);

  // the point in world coordinates
  const double ds = targetSizesMetres_[id] * 0.5;

  // These are the corners of the AprilTag represented
  // in the "tag" frame T
  Eigen::Matrix4d allPoints;
  allPoints << -ds,  ds,  ds, -ds,
               -ds, -ds,  ds,  ds,
               0.0, 0.0, 0.0, 0.0,
               1.0, 1.0, 1.0, 1.0;

  // get the right point from tag and transform to World frame
  // select the point (corner index p) for which we do the update:
  const Eigen::Vector4d hp_T = allPoints.col(p);

  // let's find the pose of the AprilTag (must have been registered):
  if(T_WT_.find(id) == T_WT_.end()){
    throw std::runtime_error("id not registered");
  }
  // let's get that transformation using this tag's (unique!) id
  // and transform the corner point into the World frame:
  const Eigen::Vector4d hp_W = T_WT_[id] * hp_T;

  // TODO: transform the corner point from world frame into the camera frame
  // (remember the camera projection will assume the point is represented
  // in camera coordinates):

  // TODO: calculate the reprojection error y (residual)
  // using the PinholeCamera::project
  const Eigen::Vector2d y;  // = TODO

  // TODO: check validity of projection -- return false if not successful!

  // TODO: calculate measurement Jacobian H

  // Obtain the measurement covariance form parameters:
  const double r = sigma_imagePoint_ * sigma_imagePoint_;
  Eigen::Matrix2d R = Eigen::Vector2d(r, r).asDiagonal();  // the measurement covariance

  // TODO: compute residual covariance S
  Eigen::Matrix2d S;  // = TODO

  // chi2 test -- See Lecture 4, Slide 21
  if (y.transpose() * S.inverse() * y > 40.0) {
    std::cout << "Rejecting measurement, residual = " << y.transpose()
        << " pixels" << std::endl;
    return false;
  }

  // TODO: compute Kalman gain K

  // TODO: compute increment Delta_x

  // TODO: perform update. Note: multiplicative for the quaternion!!

  // TODO: update to covariance matrix:

  return false;  // TODO: change to true once implemented...
}

}  // namespace arp
