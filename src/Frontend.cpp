/*
 * Frontend.cpp
 *
 *  Created on: 9 Dec 2016
 *      Author: sleutene
 */

#include <arp/Frontend.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

namespace arp {

Frontend::~Frontend() {
  if(camera_ != nullptr) {
    delete camera_;
  }
}

void Frontend::setCameraParameters(int imageWidth, int imageHeight,
                                   double focalLengthU, double focalLengthV,
                                   double imageCenterU, double imageCenterV,
                                   double k1, double k2, double p1, double p2)
{
  if(camera_ != nullptr) {
    delete camera_;
  }
  camera_ = new arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>(
          imageWidth, imageHeight, focalLengthU, focalLengthV, imageCenterU,
          imageCenterV, arp::cameras::RadialTangentialDistortion(k1, k2, p1, p2));
  camera_->initialiseUndistortMaps();
}

// the undistorted camera model used for the estimator (later)
arp::cameras::PinholeCamera<arp::cameras::NoDistortion> 
    Frontend::undistortedCameraModel() const {
  assert(camera_);
  return camera_->undistortedPinholeCamera();
}

int Frontend::detect(const cv::Mat& image, DetectionVec & detections)
{
  // TODO: implement

  const float k1 = -0.541596;
  const float k2 = 0.307486;
  const float p1 = -0.000014;
  const float p2 = 0.001816;
  const float tagSize =16.75;

  // imageWidth,imageHeight,focalLengthU,focalLengthV,imageCenterU,imageCenterV
  this->setCameraParameters(640,360,569.46,572.26,320.00,149.25, k1,k2,p1,p2);
  camera = this->undistortedCameraModel();
  camera.undistortImage(image,image);
  cv::cvtColor(image, image, CV_BGR2GRAY);
  std::vector<vpHomogeneousMatrix> cMo_vec;
  tagDetector_.detect(image,tagSize, camera, cMo_vec);
  int number_detections = detector.getNbObjects();
  for (size_t i = 0; i < number_detections; i++) {
    Detection detection; // maybe add arp:: etc.
    detection.T_CT = kinematics::Transformation(cMo_vec[i]);
    detection.points = tagDetector_.getPolygon(i);
    detection.id = i;
    }
    // std::vector<vpImagePoint> p = detector.getPolygon(i);
    // idToSize_[i] 
  



   /// \brief A simple struct containing all the necessary information about a
  ///        tag detection.
  // struct Detection {
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //   kinematics::Transformation T_CT; ///< The pose of the camera relative to the tag.
  //   Eigen::Matrix<double,2,4> points; ///< The tag corner points in pixels [u1...u4; v1...v4].
  //   int id; ///< The ID of the detected tag.
  // };
  // typedef std::vector<Detection, Eigen::aligned_allocator<Detection>> DetectionVec;



  
  
  return number_detections;
  // throw std::runtime_error("not implemented");
  // return 0; // TODO: number of detections...
}

bool Frontend::setTarget(unsigned int id, double targetSizeMeters) {
  idToSize_[id] = targetSizeMeters;
  return true;
}

}  // namespace arp

