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
  // Clear detections
  detections.clear();
  //undistort the input image and transform to grayscale
  cv::Mat undistorted_grey_image;
  cv::Mat distorted_grey_image;
  cv::cvtColor(image, distorted_grey_image, CV_BGR2GRAY);
  camera_->undistortImage(distorted_grey_image,undistorted_grey_image);
  double fu = camera_->undistortedPinholeCamera().focalLengthU();
  double fv = camera_->undistortedPinholeCamera().focalLengthV();
  double cu = camera_->undistortedPinholeCamera().imageCenterU();
  double cv = camera_->undistortedPinholeCamera().imageCenterV();

  //Extract AprilTags ie. get raw detactions using tagDetector_
  std::vector<AprilTags::TagDetection> rawdetections = tagDetector_.extractTags(undistorted_grey_image);


  for (auto const& rawdetection: rawdetections)
  {
      int detected_ID = rawdetection.id;
      if (idToSize_.find(detected_ID) == idToSize_.end())
      {
        // id not found
        continue;
      } else {
        float targetSize = idToSize_[detected_ID];
        Eigen::Matrix4d transform = rawdetection.getRelativeTransform(targetSize,fu, fv, cu, cv);
        Detection detection; // maybe add arp:: etc.
        //std::cout << "transform T_CT:" << transform;
        detection.T_CT = kinematics::Transformation(transform);
        //std::cout << "deection T_CT:" << kinematics::Transformation(transform).T();

        const std::pair<float, float> *pi = rawdetection.p;
        Eigen::Matrix<double, 2, 4> matrix;

        matrix(0,0) = pi[0].first;
        matrix(0,1) = pi[1].first;
        matrix(0,2) = pi[2].first;
        matrix(0,3) = pi[3].first;
        matrix(1,0) = pi[0].second;
        matrix(1,1) = pi[1].second;
        matrix(1,2) = pi[2].second;
        matrix(1,3) = pi[3].second;
        //std::cout<< "Matrix: " << matrix;
        detection.points = matrix;
        detection.id = detected_ID;
        detections.push_back(detection);
      }
  }
  return detections.size();
}

bool Frontend::setTarget(unsigned int id, double targetSizeMeters) {
  idToSize_[id] = targetSizeMeters;
  return true;
}

}  // namespace arp
